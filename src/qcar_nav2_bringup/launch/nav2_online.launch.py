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
    use_sim_time = LaunchConfiguration('use_sim_time')
    controller_type = LaunchConfiguration('controller_type')

    # ARGUMENTS
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_controller_type = DeclareLaunchArgument(
        'controller_type',
        default_value='rpp',
        description='Which controller to use: "rpp", "stanley", or "vector"'
    )

    # --------------------------------------------------------
    # PARAM FILE SELECTION
    # --------------------------------------------------------
    # Same three files as the navfn branch, except the navfn_planner block has
    # been replaced by hybrid_planner_server. Everything else -- controller
    # gains, costmaps, AMCL -- is identical, so a controller comparison run on
    # this branch differs from the navfn one ONLY in the planner.
    rpp_params = os.path.join(pkg_share, 'config', 'nav2_params_rpp.yaml')
    stanley_params = os.path.join(pkg_share, 'config', 'nav2_params_stanley.yaml')
    vector_params = os.path.join(pkg_share, 'config', 'nav2_params_vector_pursuit.yaml')

    # The hybrid planner is namespaced, so both the bare and the fully
    # qualified map topic have to be remapped or it never receives the map.
    planner_remaps = [('map', '/map'), ('/hybrid_planner_server/map', '/map')]

    # The hybrid planner server and the controller server are NOT lifecycle
    # nodes -- they start on their own and are deliberately left out of the
    # lifecycle manager's node_names below.
    lifecycle_nodes = [
        '/global_costmap/costmap',
        '/world_model',
        '/recoveries_server',
        '/bt_navigator',
    ]

    # ========================================================================
    # NODES (RPP SET) - Condition: controller_type == 'rpp'
    # ========================================================================
    cond_rpp = IfCondition(PythonExpression(["'", controller_type, "' == 'rpp'"]))

    global_costmap_rpp = Node(
        condition=cond_rpp,
        package='nav2_costmap_2d', node_executable='nav2_costmap_2d', node_name='costmap', node_namespace='global_costmap', output='screen',
        parameters=[rpp_params, {'use_sim_time': use_sim_time}])

    world_model_rpp = Node(
        condition=cond_rpp,
        package='nav2_world_model', node_executable='world_model', node_name='world_model', output='screen',
        parameters=[rpp_params, {'use_sim_time': use_sim_time}, {'costmap_topic': '/global_costmap/costmap_raw'}, {'footprint_topic': '/global_costmap/published_footprint'}])

    planner_rpp = Node(
        condition=cond_rpp,
        package='custom_hybrid_planner', node_executable='hybrid_planner_server', node_name='hybrid_planner_server', output='screen',
        parameters=[rpp_params, {'use_sim_time': use_sim_time}], remappings=planner_remaps)

    recoveries_rpp = Node(
        condition=cond_rpp,
        package='nav2_recoveries', node_executable='recoveries_node', node_name='recoveries_server', output='screen',
        parameters=[rpp_params, {'use_sim_time': use_sim_time}])

    bt_rpp = Node(
        condition=cond_rpp,
        package='nav2_bt_navigator', node_executable='bt_navigator', node_name='bt_navigator', output='screen',
        parameters=[rpp_params, {'use_sim_time': use_sim_time}])

    lifecycle_rpp = Node(
        condition=cond_rpp,
        package='nav2_lifecycle_manager', node_executable='lifecycle_manager', node_name='lifecycle_manager', output='screen',
        parameters=[rpp_params, {'use_sim_time': use_sim_time}, {'autostart': True},
                    {'node_names': lifecycle_nodes}])

    rpp_server = Node(
        condition=cond_rpp,
        package='rpp_controller', node_executable='rpp_controller_server', node_name='controller_server', output='screen',
        parameters=[rpp_params, {'use_sim_time': use_sim_time}])

    # ========================================================================
    # NODES (STANLEY SET) - Condition: controller_type == 'stanley'
    # ========================================================================
    cond_stanley = IfCondition(PythonExpression(["'", controller_type, "' == 'stanley'"]))

    global_costmap_stanley = Node(
        condition=cond_stanley,
        package='nav2_costmap_2d', node_executable='nav2_costmap_2d', node_name='costmap', node_namespace='global_costmap', output='screen',
        parameters=[stanley_params, {'use_sim_time': use_sim_time}])

    world_model_stanley = Node(
        condition=cond_stanley,
        package='nav2_world_model', node_executable='world_model', node_name='world_model', output='screen',
        parameters=[stanley_params, {'use_sim_time': use_sim_time}, {'costmap_topic': '/global_costmap/costmap_raw'}, {'footprint_topic': '/global_costmap/published_footprint'}])

    planner_stanley = Node(
        condition=cond_stanley,
        package='custom_hybrid_planner', node_executable='hybrid_planner_server', node_name='hybrid_planner_server', output='screen',
        parameters=[stanley_params, {'use_sim_time': use_sim_time}], remappings=planner_remaps)

    recoveries_stanley = Node(
        condition=cond_stanley,
        package='nav2_recoveries', node_executable='recoveries_node', node_name='recoveries_server', output='screen',
        parameters=[stanley_params, {'use_sim_time': use_sim_time}])

    bt_stanley = Node(
        condition=cond_stanley,
        package='nav2_bt_navigator', node_executable='bt_navigator', node_name='bt_navigator', output='screen',
        parameters=[stanley_params, {'use_sim_time': use_sim_time}])

    lifecycle_stanley = Node(
        condition=cond_stanley,
        package='nav2_lifecycle_manager', node_executable='lifecycle_manager', node_name='lifecycle_manager', output='screen',
        parameters=[stanley_params, {'use_sim_time': use_sim_time}, {'autostart': True},
                    {'node_names': lifecycle_nodes}])

    stanley_server = Node(
        condition=cond_stanley,
        package='stanley_controller', node_executable='stanley_controller_server', node_name='controller_server', output='screen',
        parameters=[stanley_params, {'use_sim_time': use_sim_time}])

    # ========================================================================
    # NODES (VECTOR PURSUIT SET) - Condition: controller_type == 'vector'
    # ========================================================================
    cond_vector = IfCondition(PythonExpression(["'", controller_type, "' == 'vector'"]))

    global_costmap_vector = Node(
        condition=cond_vector,
        package='nav2_costmap_2d', node_executable='nav2_costmap_2d', node_name='costmap', node_namespace='global_costmap', output='screen',
        parameters=[vector_params, {'use_sim_time': use_sim_time}])

    world_model_vector = Node(
        condition=cond_vector,
        package='nav2_world_model', node_executable='world_model', node_name='world_model', output='screen',
        parameters=[vector_params, {'use_sim_time': use_sim_time}, {'costmap_topic': '/global_costmap/costmap_raw'}, {'footprint_topic': '/global_costmap/published_footprint'}])

    planner_vector = Node(
        condition=cond_vector,
        package='custom_hybrid_planner', node_executable='hybrid_planner_server', node_name='hybrid_planner_server', output='screen',
        parameters=[vector_params, {'use_sim_time': use_sim_time}], remappings=planner_remaps)

    recoveries_vector = Node(
        condition=cond_vector,
        package='nav2_recoveries', node_executable='recoveries_node', node_name='recoveries_server', output='screen',
        parameters=[vector_params, {'use_sim_time': use_sim_time}])

    bt_vector = Node(
        condition=cond_vector,
        package='nav2_bt_navigator', node_executable='bt_navigator', node_name='bt_navigator', output='screen',
        parameters=[vector_params, {'use_sim_time': use_sim_time}])

    lifecycle_vector = Node(
        condition=cond_vector,
        package='nav2_lifecycle_manager', node_executable='lifecycle_manager', node_name='lifecycle_manager', output='screen',
        parameters=[vector_params, {'use_sim_time': use_sim_time}, {'autostart': True},
                    {'node_names': lifecycle_nodes}])

    vector_server = Node(
        condition=cond_vector,
        package='vector_pursuit_controller', node_executable='vector_pursuit_controller_server', node_name='controller_server', output='screen',
        parameters=[vector_params, {'use_sim_time': use_sim_time}])

    return LaunchDescription([
        declare_use_sim_time,
        declare_controller_type,
        # RPP Nodes
        global_costmap_rpp, world_model_rpp, planner_rpp, recoveries_rpp, bt_rpp, lifecycle_rpp, rpp_server,
        # Stanley Nodes
        global_costmap_stanley, world_model_stanley, planner_stanley, recoveries_stanley, bt_stanley, lifecycle_stanley, stanley_server,
        # Vector Pursuit Nodes
        global_costmap_vector, world_model_vector, planner_vector, recoveries_vector, bt_vector, lifecycle_vector, vector_server
    ])
