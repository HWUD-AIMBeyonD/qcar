from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='qcar_odom',
            node_executable='odom_node',  # Dashing syntax
            node_name='qcar_odometry_node',
            output='screen',
        )
    ])

