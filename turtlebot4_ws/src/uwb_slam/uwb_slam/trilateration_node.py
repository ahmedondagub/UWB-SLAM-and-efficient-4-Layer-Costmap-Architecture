#!/usr/bin/env python3
"""
Trilateration Node - Phase 1.2

Consumes UWB range measurements from the virtual sensor (or physical UWB hardware)
and solves for the robot's estimated position using least-squares trilateration.

Outputs:
- Estimated position (geometry_msgs/PoseStamped)
- Residual error for validation
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from typing import Optional

import numpy as np
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Float32

from uwb_slam.math_utils import Trilateration, parse_anchor_positions


class TrilaterationNode(Node):
    """
    Trilateration solver node.
    - Subscribes to UWB range measurements
    - Solves for robot position using least-squares optimization
    - Publishes estimated pose
    """
    
    def __init__(self):
        super().__init__('trilateration_node')
        
        # Declare parameters
        self.declare_parameter('anchor_positions', [-2.0, -2.0, 2.0, -2.0, 2.0, 2.0, -2.0, 2.0, 0.0, 0.0])
        self.declare_parameter('use_weighted_solve', False)
        
        # Get parameters
        anchor_list = self.get_parameter('anchor_positions').value
        self.anchor_positions = parse_anchor_positions(anchor_list)
        self.use_weighted_solve = self.get_parameter('use_weighted_solve').value
        
        # Initialize trilateration solver
        self.trilat_solver = Trilateration(self.anchor_positions)
        self.get_logger().info(f"Trilateration solver initialized with {len(self.anchor_positions)} anchors")
        
        # QoS for sensor data
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscriber: Range measurements array
        self.ranges_subscription = self.create_subscription(
            Float32MultiArray,
            '/uwb/ranges',
            self.ranges_callback,
            qos
        )
        
        # Publisher: Estimated pose
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/uwb/estimated_pose',
            qos
        )
        
        # Publisher: Solver residual error (for diagnostics)
        self.residual_pub = self.create_publisher(
            Float32,
            '/uwb/trilateration_error',
            qos
        )
        
        # Storage
        self.last_estimate: Optional[tuple] = None
        self.frame_count = 0
        
        self.get_logger().info("Trilateration node started")
    
    def ranges_callback(self, msg: Float32MultiArray):
        """
        Callback for UWB range measurements.
        Solves trilateration and publishes estimated position.
        """
        ranges = list(msg.data)
        
        if len(ranges) != len(self.anchor_positions):
            self.get_logger().warning(
                f"Range count mismatch: expected {len(self.anchor_positions)}, got {len(ranges)}"
            )
            return
        
        try:
            # Solve trilateration
            if self.use_weighted_solve:
                # Use weights based on range measurement confidence
                # Closer ranges = higher confidence
                weights = np.array([1.0 / max(r, 0.1) for r in ranges])
                weights /= weights.sum()
                
                estimated_pos = self.trilat_solver.solve_with_weights(
                    ranges, 
                    weights,
                    initial_guess=self.last_estimate if self.last_estimate else None
                )
            else:
                # Standard least-squares solve
                estimated_pos = self.trilat_solver.solve(
                    ranges,
                    initial_guess=self.last_estimate if self.last_estimate else None
                )
            
            self.last_estimate = estimated_pos
            
            # Calculate residual error
            predicted_ranges = np.array([
                np.sqrt((estimated_pos[0] - a[0])**2 + (estimated_pos[1] - a[1])**2)
                for a in self.anchor_positions[:len(ranges)]
            ])
            residual = np.mean(np.abs(predicted_ranges - np.array(ranges)))
            
            # Publish estimated pose
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            pose_msg.pose.position = Point(x=float(estimated_pos[0]), y=float(estimated_pos[1]), z=0.0)
            pose_msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            
            self.pose_pub.publish(pose_msg)
            
            # Publish residual error
            error_msg = Float32()
            error_msg.data = float(residual)
            self.residual_pub.publish(error_msg)
            
            # Log periodically
            self.frame_count += 1
            if self.frame_count % 50 == 0:
                self.get_logger().info(
                    f"Position: ({estimated_pos[0]:.2f}, {estimated_pos[1]:.2f}) | "
                    f"Error: {residual:.4f} m"
                )
        
        except Exception as e:
            self.get_logger().error(f"Trilateration solve failed: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = TrilaterationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Trilateration node")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
