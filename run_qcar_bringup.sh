#!/bin/bash

source /opt/ros/dashing/setup.bash
source ~/qcar_ws/install/setup.bash

echo "Starting QCar Unified System..."
echo "========================================"

# Start robot_description WITHOUT sudo (doesn't need hardware access)
if [ -f ~/qcar_ws/install/robot_description/share/robot_description/launch/simple_robot_launch.py ]; then
    echo "Starting robot_description (with RViz2)..."
    ros2 launch robot_description simple_robot_launch.py &
    RVIZ_PID=$!
    sleep 3
fi

# --- Option 2: Publish base -> base_footprint as a static transform (NO sudo) ---
echo "Publishing static TF: base -> base_footprint ..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base base_footprint &
BASE_FOOTPRINT_PID=$!
sleep 1

# Start hardware interface WITH sudo (needs hardware access)
echo "Starting unified hardware interface..."
sudo -E PYTHONPATH=$PYTHONPATH LD_LIBRARY_PATH=$LD_LIBRARY_PATH DISPLAY=$DISPLAY bash -c '
    source /opt/ros/dashing/setup.bash
    source ~/qcar_ws/install/setup.bash

    # Start unified hardware interface
    python3 ~/qcar_ws/install/qcar_nav2_bringup/lib/qcar_nav2_bringup/qcar_hardware_interface --ros-args -p max_speed:=0.5 -p max_steering_angle:=0.5
' &
HARDWARE_PID=$!

# (Optional) Start SLAM toolbox here when you want to include it in bringup
# Uncomment when ready:
# echo "Starting slam_toolbox (online async)..."
# ros2 launch slam_toolbox online_async_launch.py params_file:=/home/nvidia/qcar_ws/qcar_slam.yaml &
# SLAM_PID=$!

echo "========================================"
echo "QCar System Running!"
echo "Nodes active:"
echo "  ✓ robot_description (URDF, joint states, TF)"
echo "  ✓ static TF (base -> base_footprint)"
echo "  ✓ RViz2 visualization"
echo "  ✓ qcar_hardware_interface (LiDAR, odom, cmd_vel)"
# echo "  ✓ slam_toolbox (mapping)"
echo "========================================"
echo "Press Ctrl+C to stop all nodes."

# Wait for processes
trap "echo 'Stopping...';
      kill $RVIZ_PID $BASE_FOOTPRINT_PID $HARDWARE_PID 2>/dev/null;
      # kill $SLAM_PID 2>/dev/null;
      sudo pkill -f qcar_hardware_interface;
      exit" INT TERM

wait

