#!/usr/bin/env python3
"""
Localization for OptiTrack sessions -- the AMCL-free counterpart to
localization.launch.py.

OptiTrack already gives ground-truth pose, so there is nothing for a particle
filter to correct. The map->odom transform is therefore identity, and the whole
TF chain becomes:

    map --(static identity)--> odom --(http_odom_node)--> base

Provides:
    direct_map_publisher  -> /map        (Nav2 costmaps need this)
    static_transform_publisher -> TF map->odom (identity, replaces qcar_amcl)

Still needs, from elsewhere (run_qcar_nav_optitrack.sh):
    qcar_hardware_interface -> /scan, /cmd_vel
    http_odom_node          -> /odom_opti + TF odom->base
    robot_state_publisher   -> TF base->lidar

Usage:
    ros2 launch qcar_nav2_bringup localization_optitrack.launch.py
    ros2 launch qcar_nav2_bringup localization_optitrack.launch.py \\
        map:=/home/nvidia/qcar_ws/maps/final_map005.yaml
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    default_map_path = os.path.join(
        os.environ.get('HOME', '/home/nvidia'),
        'qcar_ws', 'maps', 'new_map.yaml'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_map_path = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Path to map YAML file'
    )

    map_path = LaunchConfiguration('map')

    # 1. Direct Map Publisher -- loads PGM/YAML, publishes /map for costmaps.
    #    Same node as localization.launch.py; bypasses nav2_map_server's
    #    lifecycle issues on Dashing.
    direct_map_publisher = Node(
        package='qcar_nav2_bringup',
        node_executable='direct_map_publisher',
        node_name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_path,
            'frame_id': 'map'
        }]
    )

    # 2. Static identity transform: map -> odom.
    #    OptiTrack gives ground-truth pose, so map == odom and no AMCL
    #    correction is needed.
    #    Dashing's static_transform_publisher takes positional args only:
    #        x y z yaw pitch roll frame_id child_frame_id
    #    It does NOT support --frame-id / --child-frame-id flags.
    map_to_odom_tf = Node(
        package='tf2_ros',
        node_executable='static_transform_publisher',
        node_name='map_to_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map_path,
        direct_map_publisher,
        map_to_odom_tf,
    ])
