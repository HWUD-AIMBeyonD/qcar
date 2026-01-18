from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    launch_actions = []

    # Start robot_description (URDF + TF + RViz if your launch includes it)
    robot_description_dir = get_package_share_directory('robot_description')
    launch_actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_description_dir, 'launch', 'simple_robot_launch.py')
            ),
        )
    )

    return LaunchDescription(launch_actions)





#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('qcar_nav2_bringup')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    controller_type = LaunchConfiguration('controller_type')

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

    # NEW: Switch between 'rpp' and 'stanley'
    declare_controller_type = DeclareLaunchArgument(
        'controller_type',
        default_value='rpp',
        description='Which controller to use: "rpp" or "stanley"'
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
    # CONTROLLER SELECTION (Option A vs Option B)
    # ========================================================================

    # Option A: RPP Controller
    # Runs if controller_type == 'rpp'
    rpp_server = Node(
        condition=IfCondition(PythonExpression(["'", controller_type, "' == 'rpp'"])),
        package='rpp_controller',
        node_executable='rpp_controller_server',
        node_name='controller_server',  # Matches param: controller_server
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # Option B: Stanley Controller
    # Runs if controller_type == 'stanley'
    stanley_server = Node(
        condition=IfCondition(PythonExpression(["'", controller_type, "' == 'stanley'"])),
        package='stanley_controller',
        node_executable='stanley_controller_server',
        node_name='stanley_controller_server', # Matches param: stanley_controller_server
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # Lifecycle Manager
    # Excludes controllers because they are self-managed in this Dashing workaround
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
       #         '/world_model',
                '/navfn_planner',
                '/recoveries_server',
                '/bt_navigator',
            ]},
        ],
    )
    
    # List of nodes to return
    nodes_to_run = [
        declare_params,
        declare_use_sim_time,
        declare_controller_type,
        global_costmap,
#        world_model,
        planner_server,
        recoveries_server,
        bt_navigator,
        lifecycle_manager,
        # Both are added to list, but conditions determine which one actually spawns
        rpp_server,
        stanley_server
    ]

    # ========================================================================
    # LEGACY DWB CONTROLLER (COMMENTED OUT)
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
    #     local_costmap,       # <-- Added back for DWB
    #     world_model,
    #     planner_server,
    #     controller_server,
    #     recoveries_server,
    #     bt_navigator,
    #     lifecycle_manager,
    # ]

    return LaunchDescription(nodes_to_run)
