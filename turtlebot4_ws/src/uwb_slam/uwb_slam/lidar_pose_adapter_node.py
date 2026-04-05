#!/usr/bin/env python3
"""Adapt Odometry messages into PoseWithCovarianceStamped for ESKF LiDAR updates."""

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from uwb_slam.math_utils import attach_debugger_if_requested

_WATCHDOG_STALE_SEC = 5.0


class LidarPoseAdapterNode(Node):
    """Bridge rf2o odometry output into the LiDAR pose topic expected by the ESKF."""

    def __init__(self):
        super().__init__('lidar_pose_adapter')

        self.declare_parameter('odom_topic', '/rf2o/odometry')
        self.declare_parameter('pose_topic', '/lidar/pose')

        odom_topic = str(self.get_parameter('odom_topic').value)
        pose_topic = str(self.get_parameter('pose_topic').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._last_msg_time: float | None = None

        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, pose_topic, qos)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, qos)
        # Watchdog: warn at 1 Hz when rf2o has been silent too long so that the
        # failure is attributed to this node, not to the ESKF or Nav2.
        self._odom_topic = odom_topic
        self.create_timer(1.0, self._health_watchdog)

        self.get_logger().info(f'Lidar pose adapter started | odom_topic={odom_topic}, pose_topic={pose_topic}')

    def odom_callback(self, msg: Odometry) -> None:
        self._last_msg_time = self.get_clock().now().nanoseconds * 1e-9
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = msg.header
        pose_msg.pose.pose = msg.pose.pose
        pose_msg.pose.covariance = msg.pose.covariance
        self.pose_pub.publish(pose_msg)

    def _health_watchdog(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last_msg_time is None:
            self.get_logger().warning(
                f'[watchdog] No messages received on {self._odom_topic} — '
                'rf2o_laser_odometry may not be running or /scan is silent'
            )
        elif (now - self._last_msg_time) > _WATCHDOG_STALE_SEC:
            age = now - self._last_msg_time
            self.get_logger().warning(
                f'[watchdog] {self._odom_topic} stale for {age:.1f}s — '
                'rf2o_laser_odometry may have stopped publishing'
            )


def main(args=None):
    attach_debugger_if_requested()
    rclpy.init(args=args)
    node = LidarPoseAdapterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down lidar pose adapter')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()