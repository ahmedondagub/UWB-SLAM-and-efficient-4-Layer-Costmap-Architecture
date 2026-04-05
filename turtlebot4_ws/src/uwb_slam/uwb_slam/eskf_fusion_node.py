#!/usr/bin/env python3
"""ESKF Sensor Fusion Node with dual-stack scheduling and UWB NLOS gating."""

import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, String
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

from uwb_slam.math_utils import ErrorStateKalmanFilter, parse_anchor_positions, attach_debugger_if_requested


STACK_1 = 'stack1'
STACK_2 = 'stack2'


def _yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """Convert a planar yaw angle into a quaternion."""
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


class ESKFFusionNode(Node):
    """Error-State Kalman Filter for multi-sensor localization."""

    def __init__(self):
        super().__init__('eskf_fusion')

        # Declare parameters
        self.declare_parameter('operating_stack', STACK_2)
        self.declare_parameter('process_noise_std', 0.1)
        self.declare_parameter('imu_measurement_std', 0.05)
        self.declare_parameter('odom_velocity_std', 0.08)
        self.declare_parameter('uwb_measurement_std', 0.2)
        self.declare_parameter('lidar_measurement_std', 0.15)
        self.declare_parameter('prediction_rate_hz', 100.0)
        self.declare_parameter('odom_update_rate_hz', 50.0)
        self.declare_parameter('lidar_update_rate_hz', 10.0)
        self.declare_parameter('uwb_update_rate_hz', 10.0)
        self.declare_parameter('uwb_gate_chi2_95_threshold', 3.841458820694124)
        # Require >=3 beacons by default; strict enough for trilateration, tolerant of one NLOS rejection.
        self.declare_parameter('min_uwb_beacons_required', 3)
        # Jazzy parameters do not accept nested list defaults; use a flat [x1, y1, x2, y2, ...] array.
        self.declare_parameter('anchor_positions', [-2.0, -2.0, 2.0, -2.0, 2.0, 2.0, -2.0, 2.0, 0.0, 0.0])
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('uwb_ranges_topic', '/uwb/ranges')
        self.declare_parameter('lidar_pose_topic', '/lidar/pose')  # rf2o-adapted local LiDAR odometry pose
        self.declare_parameter('imu_yaw_measurement_std', 0.08)
        self.declare_parameter('lidar_yaw_measurement_std', 0.12)

        # Get parameters
        process_noise = self.get_parameter('process_noise_std').value
        imu_noise = self.get_parameter('imu_measurement_std').value
        self.odom_velocity_std = float(self.get_parameter('odom_velocity_std').value)
        uwb_noise = self.get_parameter('uwb_measurement_std').value
        lidar_noise = self.get_parameter('lidar_measurement_std').value
        self.prediction_rate_hz = float(self.get_parameter('prediction_rate_hz').value)
        self.odom_update_rate_hz = float(self.get_parameter('odom_update_rate_hz').value)
        self.lidar_update_rate_hz = float(self.get_parameter('lidar_update_rate_hz').value)
        self.uwb_update_rate_hz = float(self.get_parameter('uwb_update_rate_hz').value)
        self.uwb_gate_threshold = float(self.get_parameter('uwb_gate_chi2_95_threshold').value)
        self.min_uwb_beacons_required = int(self.get_parameter('min_uwb_beacons_required').value)
        self.imu_yaw_measurement_std = float(self.get_parameter('imu_yaw_measurement_std').value)
        self.lidar_yaw_measurement_std = float(self.get_parameter('lidar_yaw_measurement_std').value)
        anchor_list = self.get_parameter('anchor_positions').value
        self.operating_stack = str(self.get_parameter('operating_stack').value).strip().lower()
        if self.operating_stack not in (STACK_1, STACK_2):
            self.operating_stack = STACK_2

        self.anchor_positions = parse_anchor_positions(anchor_list)

        # Initialize ESKF
        self.filter = ErrorStateKalmanFilter(
            process_noise_std=process_noise,
            imu_measurement_std=imu_noise,
            uwb_measurement_std=uwb_noise,
            lidar_measurement_std=lidar_noise
        )

        self.get_logger().info("ESKF Filter initialized")
        self.get_logger().info(
            f"Operating mode: {self.operating_stack} | Rates [pred={self.prediction_rate_hz:.1f}Hz, "
            f"odom={self.odom_update_rate_hz:.1f}Hz, lidar={self.lidar_update_rate_hz:.1f}Hz, "
            f"uwb={self.uwb_update_rate_hz:.1f}Hz]"
        )

        # QoS configuration
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Initialization guard — TF and pose are NOT published until odom has
        # arrived at least once.  Prevents Nav2 from seeing a zero-state map→odom
        # transform and acting on it before localization is actually running.
        self._odom_received = False
        self._lidar_ever_received = False  # flips True the first time /lidar/pose arrives
        self._start_ns = self.get_clock().now().nanoseconds
        self._lidar_absent_logged_at_ns = 0  # tracks last throttled lidar-absent log

        # Sensor buffers
        self.latest_imu_accel_body = None  # None until first IMU message arrives
        self.latest_imu_gyro_z = 0.0
        self.latest_imu_stamp_ns = 0
        self.latest_imu_yaw_measurement = None
        self.latest_odom_vel = None
        self.latest_odom_pose = None
        self.latest_odom_yaw = 0.0
        self.latest_lidar_pos = None
        self.latest_lidar_yaw = None
        self.latest_uwb_ranges = None
        self.last_imu_yaw_used_stamp_ns = 0
        self.last_odom_stamp_ns = 0
        self.last_odom_used_stamp_ns = 0
        self.last_lidar_stamp_ns = 0
        self.last_lidar_used_stamp_ns = 0
        self.last_uwb_stamp_ns = 0
        self.last_uwb_used_stamp_ns = 0

        # Subscribers
        imu_topic = self.get_parameter('imu_topic').value
        self.imu_sub = self.create_subscription(
            Imu,
            imu_topic,
            self.imu_callback,
            qos
        )

        odom_topic = self.get_parameter('odom_topic').value
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            qos
        )

        uwb_topic = self.get_parameter('uwb_ranges_topic').value
        self.uwb_sub = self.create_subscription(
            Float32MultiArray,
            uwb_topic,
            self.uwb_callback,
            qos
        )

        lidar_topic = self.get_parameter('lidar_pose_topic').value
        self.lidar_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            lidar_topic,
            self.lidar_callback,
            qos
        )

        # Publishers
        self.fused_pose_pub = self.create_publisher(
            PoseStamped,
            '/eskf/pose',
            qos
        )

        self.fused_odometry_pub = self.create_publisher(
            Odometry,
            '/eskf/odometry',
            qos
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        # Diagnostics publishers
        self.covariance_pub = self.create_publisher(
            Float32MultiArray,
            '/eskf/covariance_diagonal',
            qos
        )

        self.uwb_gate_diag_pub = self.create_publisher(
            Float32MultiArray,
            '/eskf/uwb_gate_diagnostics',
            qos
        )

        self.uwb_gate_status_pub = self.create_publisher(
            String,
            '/eskf/uwb_gate_status',
            qos
        )

        # Timing and state
        self.measurement_count = 0
        self.filter_step_count = 0
        self.uwb_reject_count = 0
        self.uwb_accept_count = 0

        # Strict, decoupled execution rates
        self.prediction_timer = self.create_timer(
            1.0 / max(self.prediction_rate_hz, 1e-6),
            self.prediction_tick
        )
        self.odom_timer = self.create_timer(
            1.0 / max(self.odom_update_rate_hz, 1e-6),
            self.odom_update_tick
        )
        self.lidar_timer = self.create_timer(
            1.0 / max(self.lidar_update_rate_hz, 1e-6),
            self.lidar_update_tick
        )
        self.uwb_timer = self.create_timer(
            1.0 / max(self.uwb_update_rate_hz, 1e-6),
            self.uwb_update_tick
        )
        self.publish_timer = self.create_timer(1.0 / 20.0, self.publish_outputs)
        # Health watchdog — emits a warning at 1 Hz for any sensor that has
        # not produced data within 3× its expected period.  Localises failures
        # to this node rather than letting them surface 2-3 hops downstream.
        self._watchdog_stale_sec = 3.0
        self.watchdog_timer = self.create_timer(1.0, self._health_watchdog)

        self.add_on_set_parameters_callback(self.on_set_parameters)

        self.get_logger().info("ESKF Fusion node started")
        self.get_logger().info(f"Topics: IMU={imu_topic}, Odom={odom_topic}, UWB={uwb_topic}, LiDAR={lidar_topic}")

    def _health_watchdog(self):
        """Warn at 1 Hz for any sensor that has not produced data recently."""
        now_ns = self.get_clock().now().nanoseconds
        stale_ns = int(self._watchdog_stale_sec * 1e9)

        if self.latest_imu_accel_body is None or (
            self.latest_imu_stamp_ns > 0
            and (now_ns - self.latest_imu_stamp_ns) > stale_ns
        ):
            self.get_logger().warning(
                f'[watchdog] IMU stale or never received '
                f'(last stamp {self.latest_imu_stamp_ns / 1e9:.1f}s, now {now_ns / 1e9:.1f}s)'
            )
        if not self._odom_received or (
            self.last_odom_stamp_ns > 0
            and (now_ns - self.last_odom_stamp_ns) > stale_ns
        ):
            self.get_logger().warning(
                f'[watchdog] Odometry stale or never received '
                f'(last stamp {self.last_odom_stamp_ns / 1e9:.1f}s, now {now_ns / 1e9:.1f}s)'
            )
        if self.last_lidar_stamp_ns > 0 and (now_ns - self.last_lidar_stamp_ns) > stale_ns:
            self.get_logger().warning(
                f'[watchdog] LiDAR pose stale — check rf2o and lidar_pose_adapter '
                f'(last stamp {self.last_lidar_stamp_ns / 1e9:.1f}s, now {now_ns / 1e9:.1f}s)'
            )
        elif self.last_lidar_stamp_ns == 0:
            # Lidar corrections are optional — dead reckoning is active — but
            # an absent /lidar/pose is always worth knowing about.  Throttle to
            # once per 30 s so it's audible without flooding the log.
            _throttle_ns = int(30e9)
            if (now_ns - self._lidar_absent_logged_at_ns) >= _throttle_ns:
                uptime_s = (now_ns - self._start_ns) / 1e9
                self.get_logger().warning(
                    f'[watchdog] /lidar/pose not received (uptime {uptime_s:.0f}s) — '
                    'running on IMU+Odom only. '
                    'Check rf2o_laser_odometry and lidar_pose_adapter logs.'
                )
                self._lidar_absent_logged_at_ns = now_ns
        if self.operating_stack == STACK_2 and self.last_uwb_stamp_ns == 0:
            self.get_logger().warning('[watchdog] UWB ranges never received (stack2 active)')

    def on_set_parameters(self, params):
        """Allow runtime mode switches between stack1 and stack2."""
        for param in params:
            if param.name == 'operating_stack':
                requested = str(param.value).strip().lower()
                if requested not in (STACK_1, STACK_2):
                    return SetParametersResult(
                        successful=False,
                        reason=f"operating_stack must be one of: {STACK_1}, {STACK_2}"
                    )

        for param in params:
            if param.name == 'operating_stack':
                self.operating_stack = str(param.value).strip().lower()
                self.get_logger().warn(f"Switched operating stack to: {self.operating_stack}")

        return SetParametersResult(successful=True)

    def imu_callback(self, msg: Imu):
        """Store body-frame IMU acceleration, yaw-rate, and optional yaw measurement."""
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        self.latest_imu_accel_body = (
            float(msg.linear_acceleration.x),
            float(msg.linear_acceleration.y),
        )
        self.latest_imu_gyro_z = float(msg.angular_velocity.z)
        if (qx * qx + qy * qy + qz * qz + qw * qw) > 0.5:
            self.latest_imu_yaw_measurement = math.atan2(
                2.0 * (qw * qz + qx * qy),
                1.0 - 2.0 * (qy * qy + qz * qz),
            )
        else:
            self.latest_imu_yaw_measurement = None
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        if stamp_ns <= 0:
            stamp_ns = int(self.get_clock().now().nanoseconds)
        self.latest_imu_stamp_ns = stamp_ns

    def odom_callback(self, msg: Odometry):
        """Store latest odometry velocity and pose sample."""
        self.latest_odom_vel = (
            float(msg.twist.twist.linear.x),
            float(msg.twist.twist.linear.y)
        )
        self.latest_odom_pose = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )
        q = msg.pose.pose.orientation
        self.latest_odom_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        if stamp_ns <= 0:
            stamp_ns = int(self.get_clock().now().nanoseconds)
        self.last_odom_stamp_ns = stamp_ns
        if not self._odom_received:
            self._odom_received = True
            self.get_logger().info('First odometry received — TF broadcast enabled')

    def uwb_callback(self, msg: Float32MultiArray):
        """Store latest UWB ranges; updates run on fixed-rate timer."""
        self.latest_uwb_ranges = [float(v) for v in msg.data]
        now = self.get_clock().now().nanoseconds
        self.last_uwb_stamp_ns = int(now)

    def lidar_callback(self, msg: PoseWithCovarianceStamped):
        """Store latest rf2o-derived LiDAR pose estimate; updates run on fixed-rate timer."""
        self.latest_lidar_pos = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y)
        )
        q = msg.pose.pose.orientation
        quat_norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
        if quat_norm_sq > 0.5:
            self.latest_lidar_yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
        else:
            self.latest_lidar_yaw = None
        stamp_ns = int(msg.header.stamp.sec) * 1000000000 + int(msg.header.stamp.nanosec)
        if stamp_ns <= 0:
            stamp_ns = int(self.get_clock().now().nanoseconds)
        self.last_lidar_stamp_ns = stamp_ns
        if not self._lidar_ever_received:
            self._lidar_ever_received = True
            self.get_logger().info(
                '[lidar] First /lidar/pose received — rf2o pipeline confirmed active. '
                'LiDAR corrections now folding into ESKF.'
            )

    def prediction_tick(self):
        """100 Hz prediction step using IMU body acceleration and yaw-rate."""
        if self.latest_imu_accel_body is None:
            return
        now_ns = self.get_clock().now().nanoseconds
        if self.latest_imu_stamp_ns > 0 and (now_ns - self.latest_imu_stamp_ns) > 500_000_000:
            self.get_logger().warning(
                'IMU data stale (>500 ms); skipping prediction to prevent unbounded drift'
            )
            return
        try:
            self.filter.predict(
                self.latest_imu_accel_body,
                self.latest_imu_gyro_z,
                dt=1.0 / max(self.prediction_rate_hz, 1e-6),
            )
            if self.latest_imu_yaw_measurement is not None and self.last_imu_yaw_used_stamp_ns != self.latest_imu_stamp_ns:
                self.filter.update_yaw(self.latest_imu_yaw_measurement, self.imu_yaw_measurement_std)
                self.last_imu_yaw_used_stamp_ns = self.latest_imu_stamp_ns
            self.filter_step_count += 1
        except Exception as e:
            self.get_logger().error(f"Prediction step failed: {e}")

    def odom_update_tick(self):
        """50 Hz velocity correction step (wheel odometry)."""
        if self.latest_odom_vel is None:
            return
        if self.last_odom_stamp_ns == self.last_odom_used_stamp_ns:
            return
        try:
            self.filter.update_odometry_velocity(self.latest_odom_vel, measurement_std=self.odom_velocity_std)
            self.last_odom_used_stamp_ns = self.last_odom_stamp_ns
        except Exception as e:
            self.get_logger().error(f"Odometry update failed: {e}")

    def lidar_update_tick(self):
        """10 Hz absolute pose correction step from rf2o-adapted LiDAR odometry."""
        if self.latest_lidar_pos is None:
            return
        if self.last_lidar_stamp_ns == self.last_lidar_used_stamp_ns:
            return

        try:
            self.filter.update_lidar_pose(self.latest_lidar_pos)
            if self.latest_lidar_yaw is not None:
                self.filter.update_yaw(self.latest_lidar_yaw, self.lidar_yaw_measurement_std)
            self.last_lidar_used_stamp_ns = self.last_lidar_stamp_ns
        except Exception as e:
            self.get_logger().error(f"LiDAR update failed: {e}")

    def uwb_update_tick(self):
        """10 Hz UWB range correction with per-beacon Mahalanobis gating."""
        if self.operating_stack != STACK_2:
            return
        if self.latest_uwb_ranges is None:
            return
        if self.last_uwb_stamp_ns == self.last_uwb_used_stamp_ns:
            return

        ranges = self.latest_uwb_ranges
        self.measurement_count += 1

        if len(ranges) != len(self.anchor_positions):
            self.get_logger().warn(
                f"Expected {len(self.anchor_positions)} ranges, got {len(ranges)}"
            )
            self.last_uwb_used_stamp_ns = self.last_uwb_stamp_ns
            return

        accepted_indices = []
        rejected_indices = []
        maha_values = []
        accepted_measurements = []

        try:
            # Gate each beacon independently; reject NLOS-like outliers.
            for i, (range_meas, anchor) in enumerate(zip(ranges, self.anchor_positions)):
                residual, s_scalar, _, _ = self.filter.compute_uwb_innovation(float(range_meas), tuple(anchor))
                s_scalar = max(s_scalar, 1e-12)
                maha = (residual * residual) / s_scalar
                maha_values.append(float(maha))

                if maha <= self.uwb_gate_threshold:
                    accepted_indices.append(i)
                    accepted_measurements.append((float(range_meas), tuple(anchor)))
                else:
                    rejected_indices.append(i)
                    self.uwb_reject_count += 1

            # If too few beacons pass, skip UWB correction for this cycle.
            if len(accepted_indices) < self.min_uwb_beacons_required:
                self.get_logger().warn(
                    f"UWB update under-constrained this cycle: {len(accepted_indices)} accepted < "
                    f"{self.min_uwb_beacons_required}. Relying on IMU/Odom/LiDAR."
                )
            else:
                # Apply accepted measurements only when redundancy requirement is satisfied.
                for range_meas, anchor in accepted_measurements:
                    self.filter.update_uwb(range_meas, anchor)
                    self.uwb_accept_count += 1

            self.publish_uwb_gate_diagnostics(accepted_indices, rejected_indices, maha_values)

            if self.measurement_count % 50 == 0:
                pos = self.filter.get_position()
                vel = self.filter.get_velocity()
                self.get_logger().info(
                    f"UWB Update [{self.measurement_count}]: "
                    f"Pos=({pos[0]:.2f}, {pos[1]:.2f}) "
                    f"Vel=({vel[0]:.2f}, {vel[1]:.2f}) "
                    f"Accepted={len(accepted_indices)} Rejected={len(rejected_indices)}"
                )

            self.last_uwb_used_stamp_ns = self.last_uwb_stamp_ns

        except Exception as e:
            self.get_logger().error(f"UWB update failed: {e}")

    def publish_outputs(self):
        if not self._odom_received:
            return
        self.publish_fused_pose()
        self.publish_covariance_diagnostics()

    def publish_uwb_gate_diagnostics(self, accepted_indices, rejected_indices, maha_values):
        """Publish compact diagnostics for UWB gating behavior."""
        diag = Float32MultiArray()
        # Format: [num_accepted, num_rejected, idx_0, maha_0, idx_1, maha_1, ...]
        diag_values = [float(len(accepted_indices)), float(len(rejected_indices))]
        for idx in rejected_indices:
            value = maha_values[idx] if idx < len(maha_values) else float('nan')
            diag_values.extend([float(idx), float(value)])
        diag.data = diag_values
        self.uwb_gate_diag_pub.publish(diag)

        status = String()
        status.data = (
            f"stack={self.operating_stack}, accepted={len(accepted_indices)}, "
            f"rejected={len(rejected_indices)}, threshold={self.uwb_gate_threshold:.3f}"
        )
        self.uwb_gate_status_pub.publish(status)

    def publish_fused_pose(self):
        """Publish fused pose estimate."""
        pos = self.filter.get_position()
        yaw = self.filter.get_yaw()
        now = self.get_clock().now()
        qx, qy, qz, qw = _yaw_to_quaternion(yaw)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = now.to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = float(pos[0])
        pose_msg.pose.position.y = float(pos[1])
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        self.fused_pose_pub.publish(pose_msg)

        # Also publish as Odometry
        vel = self.filter.get_velocity()
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = float(pos[0])
        odom_msg.pose.pose.position.y = float(pos[1])
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw
        odom_msg.twist.twist.linear.x = float(vel[0])
        odom_msg.twist.twist.linear.y = float(vel[1])

        # Covariance (position + yaw, from filter)
        cov = self.filter.get_covariance()
        odom_msg.pose.covariance[0] = cov[0, 0]
        odom_msg.pose.covariance[7] = cov[1, 1]
        odom_msg.pose.covariance[35] = cov[4, 4]

        self.fused_odometry_pub.publish(odom_msg)

        self.publish_map_to_odom_transform(now, pos, yaw)

    def publish_map_to_odom_transform(self, stamp, map_position, filtered_yaw):
        """Broadcast the global correction transform from map to odom."""
        map_x, map_y = map_position

        if self.latest_odom_pose is not None:
            odom_x, odom_y = self.latest_odom_pose
            yaw_diff = ErrorStateKalmanFilter.wrap_angle(filtered_yaw - self.latest_odom_yaw)
            cos_y = math.cos(yaw_diff)
            sin_y = math.sin(yaw_diff)
            tx = float(map_x - (cos_y * odom_x - sin_y * odom_y))
            ty = float(map_y - (sin_y * odom_x + cos_y * odom_y))
            rz = math.sin(yaw_diff * 0.5)
            rw = math.cos(yaw_diff * 0.5)
        else:
            # Odom not yet received — broadcast identity so TF tree is valid from startup.
            tx, ty = 0.0, 0.0
            rz, rw = 0.0, 1.0

        transform = TransformStamped()
        transform.header.stamp = stamp.to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'
        transform.transform.translation.x = tx
        transform.transform.translation.y = ty
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = rz
        transform.transform.rotation.w = rw

        self.tf_broadcaster.sendTransform(transform)

    def publish_covariance_diagnostics(self):
        """Publish covariance diagonal for diagnostics."""
        cov = self.filter.get_covariance()

        msg = Float32MultiArray()
        msg.data = [float(cov[i, i]) for i in range(cov.shape[0])]
        self.covariance_pub.publish(msg)


def main(args=None):
    attach_debugger_if_requested()
    rclpy.init(args=args)
    node = ESKFFusionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down ESKF Fusion node")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
