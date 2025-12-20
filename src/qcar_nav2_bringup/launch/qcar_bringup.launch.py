from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    launch_actions = []
    
    # Launch robot_description if it exists
    try:
        robot_description_dir = get_package_share_directory('robot_description')
        launch_actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(robot_description_dir, 'launch', 'robot_state_publisher.launch.py')
                ),
            )
        )
    except:
        pass
    
    # Launch unified hardware interface
    launch_actions.append(
        Node(
            package='qcar_nav2_bringup',
            node_executable='qcar_hardware_interface',
            node_name='qcar_hardware_interface',
            output='screen',
            parameters=[
                {'max_speed': 0.5},
                {'max_steering_angle': 0.5}
            ]
        )
    )
    
    return LaunchDescription(launch_actions)

