#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('qcar_nav2_bringup')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        description='Full path to Nav2 parameters file'
    )

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')

    # 1. Custom Hybrid Planner (Auto-Starting Node)
    # We keep the REMAPPING because it is namespaced
    hybrid_planner = Node(
        package='custom_hybrid_planner',
        node_executable='hybrid_planner_server',
        node_name='hybrid_planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('map', '/map'),
            ('/hybrid_planner_server/map', '/map')
        ]
    )

    # 2. Custom RPP Controller (Auto-Starting Node)
    # Matches your working configuration (excluded from lifecycle)
    controller_server = Node(
        package='rpp_controller',
        node_executable='rpp_controller_server',
        node_name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # 3. Standard Nav2 Nodes (These ARE Lifecycle Nodes)
    world_model = Node(
        package='nav2_world_model',
        node_executable='world_model',
        node_name='world_model',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    recoveries_server = Node(
        package='nav2_recoveries',
        node_executable='recoveries_node',
        node_name='recoveries_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        node_executable='bt_navigator',
        node_name='bt_navigator',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # 4. Lifecycle Manager
    # CRITICAL CHANGE: We REMOVE the planner and controller from this list.
    # We only keep the nodes that actually wait for the signal.
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        node_executable='lifecycle_manager',
        node_name='lifecycle_manager',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': [
		'/bt_navigator',
                '/world_model',
                '/recoveries_server'
            ]},
        ],
    )

    return LaunchDescription([
        declare_params,
        declare_use_sim_time,
        world_model,
        hybrid_planner,    # Starts automatically
        controller_server, # Starts automatically
        recoveries_server,
        bt_navigator,
        lifecycle_manager  # Manages the rest
    ])
