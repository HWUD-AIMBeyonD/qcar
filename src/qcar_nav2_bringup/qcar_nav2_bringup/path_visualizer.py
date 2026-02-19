#!/usr/bin/env python3
"""
path_visualizer.py - Display global plan waypoints
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from nav_msgs.msg import Path
import math


class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer')

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        # Subscribe to global plan
        self.plan_sub = self.create_subscription(
            Path,
            '/plan',  # Global planner publishes here
            self.plan_callback,
            qos
        )

        # Also try the transformed plan from controller
        self.local_plan_sub = self.create_subscription(
            Path,
            '/local_plan',
            self.local_plan_callback,
            qos
        )

        self.get_logger().info('Path Visualizer started - waiting for plans...')
        self.get_logger().info('Subscribed to /plan (global) and /local_plan (local)')

    def plan_callback(self, msg):
        """Handle global plan."""
        num_poses = len(msg.poses)
        self.get_logger().info(f'\n{"="*50}')
        self.get_logger().info(f'GLOBAL PLAN: {num_poses} waypoints')
        self.get_logger().info(f'{"="*50}')

        # Print every Nth waypoint to avoid spam
        step = max(1, num_poses // 10)  # Show ~10 waypoints

        for i in range(0, num_poses, step):
            pose = msg.poses[i].pose
            x = pose.position.x
            y = pose.position.y

            # Extract yaw
            q = pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            yaw_deg = math.degrees(yaw)

            self.get_logger().info(f'  [{i:3d}] x={x:7.3f}, y={y:7.3f}, yaw={yaw_deg:6.1f}deg')

        # Always show last waypoint (goal)
        if num_poses > 0:
            pose = msg.poses[-1].pose
            self.get_logger().info(f'  [GOAL] x={pose.position.x:.3f}, y={pose.position.y:.3f}')

        # Calculate total path length
        total_length = 0.0
        for i in range(1, num_poses):
            p1 = msg.poses[i-1].pose.position
            p2 = msg.poses[i].pose.position
            total_length += math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)

        self.get_logger().info(f'Total path length: {total_length:.2f} meters')

    def local_plan_callback(self, msg):
        """Handle local plan (from controller)."""
        num_poses = len(msg.poses)
        if num_poses > 0:
            self.get_logger().info(f'Local plan: {num_poses} poses', throttle_duration_sec=1.0)


def main():
    rclpy.init()
    node = PathVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

