#!/usr/bin/env python3
"""
Localization for OptiTrack sessions -- the AMCL-free counterpart to
localization.launch.py.

OptiTrack already gives ground-truth pose, so there is nothing for a particle
filter to correct. The map->odom transform is therefore CONSTANT -- but it is
not identity: it is the fixed offset between Cartographer's map origin (the
robot's pose when mapping began) and OptiTrack's origin. See map_odom_* below.

    map --(static, constant)--> odom --(http_odom_node)--> base

Provides:
    direct_map_publisher  -> /map        (Nav2 costmaps need this)
    static_transform_publisher -> TF map->odom (constant, replaces qcar_amcl)

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

    # map -> odom offset.
    #
    # This is NOT identity. Cartographer initialises its map frame at the
    # robot's pose when mapping starts, so the map origin sits at that start
    # pose -- not at OptiTrack's rig origin. For new_map, mapping began at
    # OptiTrack pose:
    #
    #     T_odom_map = (x=2.154011, y=1.927704, yaw=1.618198 rad / 92.716 deg)
    #
    # static_transform_publisher needs the opposite direction (the pose of
    # odom expressed in map), so these defaults are its inverse:
    #
    #     t' = -R(-yaw) * t ,  yaw' = -yaw
    #
    # Sanity check: map (0,0) lands on a free cell at pixel (34,245), in open
    # space next to a wall -- i.e. exactly where the car started mapping.
    #
    # Re-derive these for any NEW map: read /odom_opti at the instant mapping
    # starts, then invert as above.
    declare_map_odom_x = DeclareLaunchArgument(
        'map_odom_x',
        default_value='-1.823473',
        description='X of the odom origin expressed in the map frame'
    )

    declare_map_odom_y = DeclareLaunchArgument(
        'map_odom_y',
        default_value='2.242934',
        description='Y of the odom origin expressed in the map frame'
    )

    declare_map_odom_yaw = DeclareLaunchArgument(
        'map_odom_yaw',
        default_value='-1.618198',
        description='Yaw of the odom origin in the map frame, in RADIANS'
    )

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

    # 2. Static transform: map -> odom.
    #    OptiTrack gives ground-truth pose, so this transform is constant and
    #    no AMCL correction is needed -- but it is NOT identity unless the map
    #    happens to have been built from OptiTrack's origin. See map_odom_*.
    #    Dashing's static_transform_publisher takes positional args only:
    #        x y z yaw pitch roll frame_id child_frame_id
    #    It does NOT support --frame-id / --child-frame-id flags.
    map_to_odom_tf = Node(
        package='tf2_ros',
        node_executable='static_transform_publisher',
        node_name='map_to_odom',
        output='screen',
        arguments=[
            LaunchConfiguration('map_odom_x'),
            LaunchConfiguration('map_odom_y'),
            '0',
            LaunchConfiguration('map_odom_yaw'),
            '0',
            '0',
            'map',
            'odom',
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map_path,
        declare_map_odom_x,
        declare_map_odom_y,
        declare_map_odom_yaw,
        direct_map_publisher,
        map_to_odom_tf,
    ])

