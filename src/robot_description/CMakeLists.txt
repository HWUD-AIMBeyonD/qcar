cmake_minimum_required(VERSION 3.8)
project(robot_description)


find_package(ament_cmake REQUIRED)

find_package(xacro REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_control REQUIRED)
find_package(controller_manager REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(robot_state_publisher REQUIRED) 
 


install(DIRECTORY urdf worlds models meshes launch rviz
  DESTINATION share/${PROJECT_NAME})


ament_package()
