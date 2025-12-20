from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='qcar_teleop',
            node_executable='cmd_vel_to_qcar',  # Dashing uses node_executable
            node_name='cmd_vel_to_qcar',        # Dashing uses node_name
            output='screen',
            parameters=[
                {'max_speed': 0.5},
                {'max_steering_angle': 0.5}
            ]
        )
    ])

