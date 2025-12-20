#!/bin/bash

source /opt/ros/dashing/setup.bash
source ~/qcar_ws/install/setup.bash

# Run with sudo while preserving environment
sudo -E PYTHONPATH=$PYTHONPATH LD_LIBRARY_PATH=$LD_LIBRARY_PATH \
     python3 ~/qcar_ws/install/qcar_odom/lib/qcar_odom/odom_node \
     --ros-args

