cmake_minimum_required(VERSION 3.5)
project(custom_hybrid_planner)

# Default to C++14 for ROS 2 Dashing
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(builtin_interfaces REQUIRED)
#find_package(nav2_core REQUIRED) 

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  nav2_costmap_2d
  nav2_msgs
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  geometry_msgs
  pluginlib
  builtin_interfaces
 # nav2_core
)

# 1. Build the Hybrid Planner Library
add_library(${PROJECT_NAME} SHARED
  src/hybrid_planner.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# 2. Build the Standalone Server Executable
add_executable(hybrid_planner_server
  src/hybrid_planner_server.cpp
)

target_include_directories(hybrid_planner_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link the server to the planner library
target_link_libraries(hybrid_planner_server ${PROJECT_NAME})
ament_target_dependencies(hybrid_planner_server ${dependencies})

# 3. Installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS hybrid_planner_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES custom_hybrid_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
