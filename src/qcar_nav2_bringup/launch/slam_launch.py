import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('qcar_nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')

    default_slam_params = os.path.join(
        pkg_share, 'config', 'slam_toolbox_params.yaml'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_slam_params = DeclareLaunchArgument(
        'slam_params_file',
        default_value=default_slam_params,
        description='Full path to the SLAM Toolbox YAML config file'
    )

    slam_params_file = LaunchConfiguration('slam_params_file')

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/scan', '/scan')
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_slam_params,
        slam_toolbox_node,
        rviz_node
    ])

