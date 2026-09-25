#!/usr/bin/env python3
"""
Sensor Fusion Launch File for QCar

Launches:
  - Simple EKF node (fuses /odom_raw + /imu)

Topics:
  Input:
    - /odom_raw: Wheel encoder odometry from qcar_hardware_interface
    - /imu: IMU data from qcar_hardware_interface

  Output:
    - /odometry/filtered: Fused odometry
    - /path_fused: Fused path for RViz
    - /tf: odom -> base transform
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Simple EKF node (replaces robot_localization)
        Node(
            package='qcar_odom',
            node_executable='simple_ekf',
            node_name='simple_ekf',
            output='screen',
        ),
    ])
