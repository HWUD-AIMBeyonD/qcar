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

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Costmaps (Dashing executable is "nav2_costmap_2d")
    global_costmap = Node(
        package='nav2_costmap_2d',
        node_executable='nav2_costmap_2d',
        node_name='costmap',
        node_namespace='global_costmap',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    local_costmap = Node(
        package='nav2_costmap_2d',
        node_executable='nav2_costmap_2d',
        node_name='costmap',
        node_namespace='local_costmap',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    world_model = Node(
         package='nav2_world_model',
         node_executable='world_model',
         node_name='world_model',
         output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
            # These topics exist in your graph already:
            {'costmap_topic': '/global_costmap/costmap_raw'},
            {'footprint_topic': '/global_costmap/published_footprint'},
        ],
    )

    planner_server = Node(
        package='nav2_navfn_planner',
        node_executable='navfn_planner',
        node_name='navfn_planner',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('get_costmap', '/GetCostmap')],
    )

    controller_server = Node(
        package='dwb_controller',
        node_executable='dwb_controller',
        node_name='controller_server',
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

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        node_executable='lifecycle_manager',
        node_name='lifecycle_manager',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
            {
                'autostart': True,
                'node_names': [
                    'global_costmap/costmap',
                    'local_costmap/costmap',
                    'world_model',
                    'navfn_planner',
                    'controller_server',
                    'recoveries_server',
                    'bt_navigator',
                ],
            },
        ],
    )

    return LaunchDescription([
        declare_params,
        declare_use_sim_time,
        global_costmap,
        local_costmap,
        world_model,
        planner_server,
        controller_server,
        recoveries_server,
        bt_navigator,
        lifecycle_manager,
    ])

