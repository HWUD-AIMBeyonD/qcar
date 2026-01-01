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

    # ========================================================================
    # COMMON NODES (Always Used)
    # ========================================================================

    global_costmap = Node(
        package='nav2_costmap_2d',
        node_executable='nav2_costmap_2d',
        node_name='costmap',
        node_namespace='global_costmap',
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

    # ========================================================================
    # OPTION A: CUSTOM RPP CONTROLLER (ACTIVE)
    # Use this block for your custom 'rpp_controller_server'
    # ========================================================================

    # 1. Controller: Uses your custom package and executable
    controller_server = Node(
        package='rpp_controller',
        node_executable='rpp_controller_server',
        node_name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # 2. Lifecycle Manager: EXCLUDES controller_server and local_costmap
    # (Because RPP starts automatically and manages its own internal costmap)
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
                '/global_costmap/costmap',
                # '/local_costmap/costmap',  <-- Removed for RPP
                '/world_model',
                '/navfn_planner',
                # '/controller_server',      <-- Removed for RPP
                '/bt_navigator',
            ]},
        ],
    )
    
    # 3. List of nodes to return for Option A
    # Note: 'local_costmap' node is MISSING here on purpose (it's inside RPP)
    nodes_to_run = [
        declare_params,
        declare_use_sim_time,
        global_costmap,
        world_model,
        planner_server,
        controller_server,
        recoveries_server,
        bt_navigator,
        lifecycle_manager,
    ]

    # ========================================================================
    # OPTION B: LEGACY DWB CONTROLLER (COMMENTED OUT)
    # Uncomment this block (and comment out Option A) to revert to DWB
    # ========================================================================

    # # 1. Controller: Uses standard Dashing package
    # controller_server = Node(
    #     package='dwb_controller',
    #     node_executable='dwb_controller',
    #     node_name='controller_server',
    #     output='screen',
    #     parameters=[params_file, {'use_sim_time': use_sim_time}],
    # )

    # # 2. Local Costmap: Needs a STANDALONE node for DWB
    # local_costmap = Node(
    #     package='nav2_costmap_2d',
    #     node_executable='nav2_costmap_2d',
    #     node_name='costmap',
    #     node_namespace='local_costmap',
    #     output='screen',
    #     parameters=[params_file, {'use_sim_time': use_sim_time}],
    # )

    # # 3. Lifecycle Manager: INCLUDES everything
    # lifecycle_manager = Node(
    #     package='nav2_lifecycle_manager',
    #     node_executable='lifecycle_manager',
    #     node_name='lifecycle_manager',
    #     output='screen',
    #     parameters=[
    #         params_file,
    #         {'use_sim_time': use_sim_time},
    #         {'autostart': True},
    #         {'node_names': [
    #             '/global_costmap/costmap',
    #             '/local_costmap/costmap',
    #             '/world_model',
    #             '/navfn_planner',
    #             '/controller_server',
    #             '/bt_navigator',
    #         ]},
    #     ],
    # )

    # # 4. List of nodes to return for Option B
    # nodes_to_run = [
    #     declare_params,
    #     declare_use_sim_time,
    #     global_costmap,
    #     local_costmap,      # <-- Added back for DWB
    #     world_model,
    #     planner_server,
    #     controller_server,
    #     recoveries_server,
    #     bt_navigator,
    #     lifecycle_manager,
    # ]

    return LaunchDescription(nodes_to_run)
