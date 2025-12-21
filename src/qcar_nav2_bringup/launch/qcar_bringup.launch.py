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

