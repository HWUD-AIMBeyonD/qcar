#!/usr/bin/env python3
"""
Cartographer SLAM driven by OptiTrack ground-truth odometry.

Starts http_odom_node alongside Cartographer, so this one launch file
provides the whole odom side of the pipeline:

    OptiTrack (HTTP) -> http_odom_node -> /odom_opti + TF odom->base
    cartographer_node                  -> /map + TF map->odom

Still needs, from elsewhere:
    qcar_hardware_interface   -> /scan   (run_qcar_mapping_optitrack.sh)
    robot_state_publisher     -> TF base->lidar

Usage:
    ros2 launch qcar_nav2_bringup cartographer_optitrack.launch.py
    ros2 launch qcar_nav2_bringup cartographer_optitrack.launch.py \\
        pose_url:=http://192.168.0.5:8000/QCar/pose
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Define the package name
    pkg_share = get_package_share_directory('qcar_nav2_bringup')

    # Create launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    configuration_directory = LaunchConfiguration('configuration_directory')
    configuration_basename = LaunchConfiguration('configuration_basename')

    # Default config directory and file
    default_config_dir = os.path.join(pkg_share, 'config')
    default_config_basename = 'qcar_cartographer.lua'

    # Declare the launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_configuration_directory = DeclareLaunchArgument(
        'configuration_directory',
        default_value=default_config_dir,
        description='Full path to config directory containing .lua files'
    )

    declare_configuration_basename = DeclareLaunchArgument(
        'configuration_basename',
        default_value=default_config_basename,
        description='Name of the Cartographer .lua configuration file'
    )

    declare_pose_url = DeclareLaunchArgument(
        'pose_url',
        default_value='http://192.168.0.3:8000/QCar/pose',
        description='OptiTrack HTTP endpoint returning {"x", "y", "yaw"} JSON'
    )

    # OptiTrack HTTP odometry -- publishes /odom_opti and TF odom->base.
    # Started first so the transform exists before Cartographer looks it up.
    http_odom_node = Node(
        package='qcar_http_odom',
        node_executable='http_odom_node',
        node_name='http_odom_node',
        output='screen',
        parameters=[{
            'pose_url': LaunchConfiguration('pose_url'),
            'rate_hz': 50.0,
            'request_timeout': 0.1,
            'odom_frame': 'odom',
            'child_frame': 'base',
            'publish_tf': True,
        }],
    )

    # Cartographer SLAM node
    cartographer_node = Node(
        package='cartographer_ros',
        node_executable='cartographer_node',
        node_name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', configuration_directory,
            '-configuration_basename', configuration_basename
        ],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odom_opti'),  # OptiTrack ground-truth odometry
            # ('imu', '/imu') # Removed since we aren't using it in Cartographer directly
        ]
    )

    # Occupancy grid node (publishes /map for Nav2/RViz)
    occupancy_grid_node = Node(
        package='cartographer_ros',
        node_executable='occupancy_grid_node', # Fixed typo here
        node_name='occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-resolution', '0.05',
            '-publish_period_sec', '1.0'
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_configuration_directory,
        declare_configuration_basename,
        declare_pose_url,
        http_odom_node,
        cartographer_node,
        occupancy_grid_node
    ])
