import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('qcar_nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    default_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Full path to the Nav2 parameters file'
    )

    # ----------------------------
    # Costmaps (namespaced)
    # NOTE: In Dashing, use node_namespace (NOT namespace=)
    # Node name is "costmap" inside each namespace.
    # ----------------------------
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

    # ----------------------------
    # Planner (Navfn)
    # ----------------------------
    planner_server = Node(
        package='nav2_navfn_planner',
        node_executable='navfn_planner',
        node_name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # ----------------------------
    # Controller (DWB)
    # ----------------------------
    controller_server = Node(
        package='dwb_controller',
        node_executable='dwb_controller',
        node_name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # ----------------------------
    # Recoveries (Dashing executable is recoveries_node)
    # ----------------------------
    recoveries_server = Node(
        package='nav2_recoveries',
        node_executable='recoveries_node',
        node_name='recoveries_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # ----------------------------
    # BT Navigator
    # ----------------------------
    bt_navigator = Node(
        package='nav2_bt_navigator',
        node_executable='bt_navigator',
        node_name='bt_navigator',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # ----------------------------
    # Lifecycle Manager
    # ----------------------------
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        node_executable='lifecycle_manager',
        node_name='lifecycle_manager',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        global_costmap,
        local_costmap,
        planner_server,
        controller_server,
        recoveries_server,
        bt_navigator,
        lifecycle_manager
    ])

