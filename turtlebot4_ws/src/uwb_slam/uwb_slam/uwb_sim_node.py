#!/usr/bin/env python3
"""
UWB Virtual Sensor Node - Phase 1.2

Extracts ground truth robot position from Gazebo Harmonic using the
odometry-publisher system plugin and generates synthetic noisy UWB range
measurements from 4 fixed anchors.

This node bridges simulation and the UWB SLAM system, providing:
1. Ground truth position from Gazebo
2. Noisy range measurements to 4 fixed anchors
3. ROS 2 topics for trilateration solver
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import Range
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import math


class UWBVirtualSensor(Node):
    """
    Virtual UWB sensor that:
    - Listens to Gazebo ground truth odometry
    - Calculates distance to 4 fixed anchors
    - Adds realistic NLOS noise (Non-Line-of-Sight)
    - Publishes range measurements and ground truth for validation
    """
    
    def __init__(self):
        super().__init__('uwb_virtual_sensor')
        
        # Declare parameters
        self.declare_parameter('anchor_positions', [-2.0, -2.0, 2.0, -2.0, 2.0, 2.0, -2.0, 2.0, 0.0, 0.0])
        self.declare_parameter('range_noise_std', 0.1)  # meters
        self.declare_parameter('range_bias_max', 0.05)  # NLOS bias
        self.declare_parameter('max_range', 10.0)  # Maximum detection range
        self.declare_parameter('ground_truth_pose_topic', '/model/turtlebot4/odometry_publisher/tf')
        self.declare_parameter('ground_truth_odom_topic', '/odom')
        
        # Get parameters
        anchor_list = self.get_parameter('anchor_positions').value
        if isinstance(anchor_list, str):
            import ast
            try:
                anchor_list = ast.literal_eval(anchor_list)
            except (ValueError, SyntaxError) as e:
                raise ValueError(f'anchor_positions: failed to parse string value: {e}') from e
        anchor_array = np.array(anchor_list, dtype=float)
        if anchor_array.ndim == 1:
            if anchor_array.size < 6 or (anchor_array.size % 2) != 0:
                raise ValueError('anchor_positions must be [x1, y1, x2, y2, ...] with at least 3 anchors')
            anchor_array = anchor_array.reshape((-1, 2))
        elif not (anchor_array.ndim == 2 and anchor_array.shape[1] == 2):
            raise ValueError('anchor_positions must be Nx2 or flat [x1, y1, x2, y2, ...]')
        self.anchor_positions = anchor_array
        self.range_noise_std = self.get_parameter('range_noise_std').value
        self.range_bias_max = self.get_parameter('range_bias_max').value
        self.max_range = self.get_parameter('max_range').value
        
        self.num_anchors = len(self.anchor_positions)
        self.get_logger().info(f"UWB Virtual Sensor initialized with {self.num_anchors} anchors")
        self.get_logger().info(f"Anchor positions: {self.anchor_positions}")
        
        # QoS profile for sensor data (best effort, fast)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers: Ground truth from Gazebo. Use whichever stream is available.
        pose_topic = str(self.get_parameter('ground_truth_pose_topic').value)
        odom_topic = str(self.get_parameter('ground_truth_odom_topic').value)

        self.pose_subscription = self.create_subscription(
            PoseStamped,
            pose_topic,
            self.pose_callback,
            qos
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            qos
        )
        
        # Publishers: Range measurements (one per anchor)
        self.range_publishers = []
        for i in range(self.num_anchors):
            pub = self.create_publisher(
                Range,
                f'/uwb/range_{i}',
                qos
            )
            self.range_publishers.append(pub)
        
        # Publisher: All ranges as array for trilateration node
        self.ranges_array_pub = self.create_publisher(
            Float32MultiArray,
            '/uwb/ranges',
            qos
        )
        
        # Publisher: Ground truth for validation
        self.ground_truth_pub = self.create_publisher(
            PoseStamped,
            '/uwb/ground_truth',
            qos
        )
        
        # Storage for current pose
        self.current_pose = None
        self.frame_count = 0
        self._last_pose_cb_ns = 0  # tracks last time pose_callback was invoked
        
        self.get_logger().info("UWB Virtual Sensor node started")
        self.get_logger().info(f"Ground truth topics: Pose={pose_topic}, Odom={odom_topic}")

    def odom_callback(self, msg: Odometry):
        """Convert odometry pose to PoseStamped; used only when dedicated pose topic is absent."""
        now_ns = self.get_clock().now().nanoseconds
        if (now_ns - self._last_pose_cb_ns) < 100_000_000:  # pose topic active within last 100 ms
            return
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.pose = msg.pose.pose
        self.pose_callback(pose_msg)
    
    def pose_callback(self, msg: PoseStamped):
        """
        Callback for Gazebo ground truth pose.

        Calculates ranges to all anchors with realistic noise model:
        - Gaussian measurement noise (LOS)
        - Occasional NLOS bias (Non-Line-of-Sight)
        """
        self._last_pose_cb_ns = self.get_clock().now().nanoseconds
        self.current_pose = msg
        
        # Extract robot position
        robot_x = msg.pose.position.x
        robot_y = msg.pose.position.y
        robot_pos = np.array([robot_x, robot_y])
        
        # Calculate distances to anchors
        ranges = []
        range_msg_array = Float32MultiArray()
        
        for i, anchor_pos in enumerate(self.anchor_positions):
            # True distance
            true_distance = np.linalg.norm(robot_pos - anchor_pos)
            
            if true_distance > self.max_range:
                self.get_logger().warning(
                    f"Anchor {i} out of range: {true_distance:.2f}m > {self.max_range}m (clamped)"
                )
                ranges.append(self.max_range)
                # Still publish per-anchor Range so subscribers and health monitors stay active
                range_msg_oor = Range()
                range_msg_oor.header.stamp = self.get_clock().now().to_msg()
                range_msg_oor.header.frame_id = 'uwb_frame'
                range_msg_oor.radiation_type = Range.INFRARED  # UWB is EM, not acoustic
                range_msg_oor.field_of_view = 2 * math.pi
                range_msg_oor.min_range = 0.1
                range_msg_oor.max_range = self.max_range
                range_msg_oor.range = float(self.max_range)
                self.range_publishers[i].publish(range_msg_oor)
                continue
            
            # Add measurement noise (Gaussian)
            measurement_noise = np.random.normal(0, self.range_noise_std)
            
            # Add occasional NLOS bias (20% chance)
            nlos_bias = 0.0
            if np.random.random() < 0.2:  # 20% NLOS probability
                nlos_bias = np.random.uniform(0, self.range_bias_max)
            
            # Final noisy range measurement
            noisy_range = true_distance + measurement_noise + nlos_bias
            
            # Clamp to physical bounds
            noisy_range = max(0.0, noisy_range)
            ranges.append(noisy_range)
            
            # Publish individual range measurement
            range_msg = Range()
            range_msg.header.stamp = self.get_clock().now().to_msg()
            range_msg.header.frame_id = 'uwb_frame'
            range_msg.radiation_type = Range.INFRARED  # UWB is EM, not acoustic
            range_msg.field_of_view = 2 * math.pi  # Omnidirectional
            range_msg.min_range = 0.1
            range_msg.max_range = self.max_range
            range_msg.range = float(noisy_range)
            
            self.range_publishers[i].publish(range_msg)
        
        # Publish range array
        range_msg_array.data = [float(r) for r in ranges]
        self.ranges_array_pub.publish(range_msg_array)
        
        # Publish ground truth for validation
        self.ground_truth_pub.publish(msg)
        
        # Log periodically
        self.frame_count += 1
        if self.frame_count % 50 == 0:  # Every 50 frames
            self.get_logger().info(
                f"Robot: ({robot_x:.2f}, {robot_y:.2f}) | "
                f"Ranges: {[f'{r:.2f}' for r in ranges]} m"
            )


def main(args=None):
    rclpy.init(args=args)
    node = UWBVirtualSensor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down UWB Virtual Sensor")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
