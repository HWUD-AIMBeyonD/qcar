#!/bin/bash

# Source ROS2 environment
source /opt/ros/dashing/setup.bash
source ~/qcar_ws/install/setup.bash

# Run the node with sudo while preserving environment
sudo -E PYTHONPATH=$PYTHONPATH LD_LIBRARY_PATH=$LD_LIBRARY_PATH \
     python3 ~/qcar_ws/install/qcar_teleop/lib/qcar_teleop/cmd_vel_to_qcar \
     --ros-args -p max_speed:=0.5 -p max_steering_angle:=0.5

