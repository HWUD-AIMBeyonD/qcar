#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('qcar_nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Hardcoded map file
    default_map_path = os.path.join(
        os.environ.get('HOME', '/home/nvidia'),
        'qcar_ws', 'maps', 'final_map005.yaml'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # 1. Direct Map Publisher - bypasses nav2_map_server lifecycle issues
    direct_map_publisher = Node(
        package='qcar_nav2_bringup',
        node_executable='direct_map_publisher',
        node_name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': default_map_path,
            'frame_id': 'map'
        }]
    )

    # 2. Custom AMCL - our own implementation, no lifecycle manager needed
    amcl = Node(
        package='qcar_amcl',
        node_executable='amcl_node',
        node_name='amcl',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'base_frame_id': 'base',
            'odom_frame_id': 'odom',
            'global_frame_id': 'map',
            'scan_topic': '/scan',
            'num_particles': 500,
            'update_rate': 10.0,
            'sigma_hit': 0.2,
            'scan_subsample': 10,
            'z_hit': 0.95,
            'z_rand': 0.05,
            'update_min_d': 0.1,
            'update_min_a': 0.15,
            'alpha1': 0.2,
            'alpha2': 0.2,
            'alpha3': 0.2,
            'alpha4': 0.2,
            'random_particle_pct': 0.05,
        }]
    )

    # No lifecycle manager needed - both nodes are regular nodes now

    return LaunchDescription([
        declare_use_sim_time,
        direct_map_publisher,
        amcl,
    ])
