#!/usr/bin/env python3
"""
Standalone HTTP odometry launch file for QCar.

Polls a JSON pose endpoint ({"x": .., "y": .., "yaw": ..}, yaw in degrees)
and publishes:
  - /odom_opti  (nav_msgs/Odometry, odom -> base)
  - /tf         (odom -> base), unless publish_tf is turned off

NOTE: qcar_odom's simple_ekf also broadcasts odom -> base. Run only one of
them, or start this node directly with -p publish_tf:=false (see below).

Usage:
  ros2 launch qcar_http_odom http_odom.launch.py
  ros2 launch qcar_http_odom http_odom.launch.py pose_url:=http://192.168.0.5:8000/QCar/pose

Or without launch, for full parameter control:
  ros2 run qcar_http_odom http_odom_node --ros-args -p publish_tf:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pose_url = LaunchConfiguration('pose_url')

    return LaunchDescription([
        DeclareLaunchArgument(
            'pose_url',
            default_value='http://192.168.0.3:8000/QCar/pose',
            description='HTTP endpoint returning {"x", "y", "yaw"} JSON'),

        Node(
            package='qcar_http_odom',
            # Dashing spells these node_executable/node_name; they were
            # renamed to executable/name in Foxy
            node_executable='http_odom_node',
            node_name='http_odom_node',
            output='screen',
            parameters=[{
                'pose_url': pose_url,
                'rate_hz': 50.0,
                'request_timeout': 0.1,
                'odom_frame': 'odom',
                'child_frame': 'base',
                'publish_tf': True,
                # OptiTrack's zero heading is 90 deg off ROS's +X-forward
                # convention. Flip to -90.0 if the car ends up facing left.
                'yaw_offset_deg': 90.0,
            }],
        ),
    ])

