from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='qcar_lidar',
            node_executable='lidar_node',
            node_name='qcar_lidar_node',
            output='screen',
        )
    ])

