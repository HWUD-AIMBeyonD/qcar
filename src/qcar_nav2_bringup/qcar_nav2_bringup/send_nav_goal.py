#!/usr/bin/env python3
"""
send_nav_goal.py - Send precise navigation goals to Nav2
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math
import sys


class NavGoalSender(Node):
    def __init__(self):
        super().__init__('nav_goal_sender')

        # Action client for Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info('Waiting for Nav2 action server...')
        self.nav_client.wait_for_server()
        self.get_logger().info('Nav2 action server connected!')

    def send_goal(self, x, y, yaw_degrees=0.0):
        """Send a navigation goal."""

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Position
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0

        # Orientation (yaw in degrees -> quaternion)
        yaw = math.radians(yaw_degrees)
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f'Sending goal: x={x}, y={y}, yaw={yaw_degrees}deg')

        # Send goal
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return

        self.get_logger().info('Goal accepted! Navigating...')

        # Wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        current_pose = feedback.current_pose.pose
        x = current_pose.position.x
        y = current_pose.position.y
        self.get_logger().info(f'Current position: ({x:.2f}, {y:.2f})')

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation complete!')


def main():
    rclpy.init()
    node = NavGoalSender()

    # Default goal or from command line
    if len(sys.argv) >= 3:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        yaw = float(sys.argv[3]) if len(sys.argv) >= 4 else 0.0
    else:
        # Default test goal - CHANGE THIS to a point on your map
        x = 2.0
        y = 0.0
        yaw = 0.0
        print(f"Usage: ros2 run qcar_nav2_bringup send_nav_goal <x> <y> [yaw_degrees]")
        print(f"Using default goal: ({x}, {y}, {yaw}deg)")

    node.send_goal(x, y, yaw)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

