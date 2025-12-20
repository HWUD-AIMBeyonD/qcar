#!/usr/bin/env python3
import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('robot_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'qcar_model.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    urdf_temp_path = '/tmp/robot_description.urdf'
    with open(urdf_temp_path, 'w') as f:
        f.write(robot_description)
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[urdf_temp_path]
    )

    # Joint state publisher (required for continuous/revolute joints)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        node_executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

    # RViz
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'simple_robot.rviz')
    rviz_node = Node(
        package='rviz2',
        node_executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])

