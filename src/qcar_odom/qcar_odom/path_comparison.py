#!/usr/bin/env python3
"""
Fused Path Publisher Node for QCar

Subscribes to:
  - /odometry/filtered: EKF fused odometry from robot_localization

Publishes:
  - /path_fused: Path from EKF fusion (for RViz visualization)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import math


class FusedPathNode(Node):
    def __init__(self):
        super().__init__('fused_path_node')

        # Path publisher
        self.path_fused_pub = self.create_publisher(Path, '/path_fused', 10)

        # Initialize path
        self.path_fused = Path()
        self.path_fused.header.frame_id = 'odom'

        # Track last position for minimum distance filtering
        self.last_fused_x = 0.0
        self.last_fused_y = 0.0
        self.min_distance = 0.02  # 2cm minimum movement

        # Subscriber for EKF fused odometry
        self.odom_filtered_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_filtered_callback,
            10
        )

        # Publish path at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_path)

        self.get_logger().info('Fused Path Node started')
        self.get_logger().info('  /path_fused - EKF fused trajectory')

    def odom_filtered_callback(self, msg):
        """Handle EKF fused odometry."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Check minimum distance
        dx = x - self.last_fused_x
        dy = y - self.last_fused_y
        if math.sqrt(dx*dx + dy*dy) >= self.min_distance:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose = msg.pose.pose
            self.path_fused.poses.append(pose)
            self.last_fused_x = x
            self.last_fused_y = y

    def publish_path(self):
        """Publish the fused path."""
        self.path_fused.header.stamp = self.get_clock().now().to_msg()
        self.path_fused_pub.publish(self.path_fused)


def main(args=None):
    rclpy.init(args=args)
    node = FusedPathNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
