#!/bin/bash
# QCar Mapping Mode: Bringup + SLAM + Teleop
# Use this to drive around and create a map

source /opt/ros/dashing/setup.bash
source ~/qcar_ws/install/setup.bash

echo "=========================================="
echo "Starting QCar MAPPING System..."
echo "=========================================="

# Clean up any existing hardware interface instances
echo "Checking for existing hardware interfaces..."
sudo pkill -f qcar_hardware_interface 2>/dev/null
sudo pkill -f cmd_vel_to_qcar 2>/dev/null
sleep 2
echo "✓ Cleanup complete"
echo ""

# Start robot_description WITHOUT sudo (doesn't need hardware access)
if [ -f ~/qcar_ws/install/robot_description/share/robot_description/launch/simple_robot_launch.py ]; then
    echo "✓ Starting robot_description (with RViz2)..."
    ros2 launch robot_description simple_robot_launch.py &
    RVIZ_PID=$!
    sleep 3
fi

# Start hardware interface WITH sudo (needs hardware access)
echo "✓ Starting unified hardware interface..."
sudo -E PYTHONPATH=$PYTHONPATH LD_LIBRARY_PATH=$LD_LIBRARY_PATH DISPLAY=$DISPLAY bash -c '
    source /opt/ros/dashing/setup.bash
    source ~/qcar_ws/install/setup.bash
    python3 ~/qcar_ws/install/qcar_nav2_bringup/lib/qcar_nav2_bringup/qcar_hardware_interface --ros-args -p max_speed:=0.5 -p max_steering_angle:=0.5
' &
HARDWARE_PID=$!
sleep 2

# Start SLAM Toolbox
echo "✓ Starting slam_toolbox (online mapping)..."
ros2 launch qcar_nav2_bringup slam_launch.launch.py &
SLAM_PID=$!
sleep 3

# Start Teleop for manual driving
echo "✓ Starting teleop control..."
sudo -E PYTHONPATH=$PYTHONPATH LD_LIBRARY_PATH=$LD_LIBRARY_PATH bash -c '
    source /opt/ros/dashing/setup.bash
    source ~/qcar_ws/install/setup.bash
    python3 ~/qcar_ws/install/qcar_teleop/lib/qcar_teleop/cmd_vel_to_qcar --ros-args -p max_speed:=0.5 -p max_steering_angle:=0.5
' &
TELEOP_PID=$!

echo "=========================================="
echo "QCar MAPPING System Running!"
echo "=========================================="
echo "Active components:"
echo "  ✓ robot_description (URDF, TF)"
echo "  ✓ RViz2 visualization"
echo "  ✓ qcar_hardware_interface (LiDAR, odom, cmd_vel)"
echo "  ✓ slam_toolbox (building map)"
echo "  ✓ teleop control (keyboard/gamepad)"
echo "=========================================="
echo ""
echo "INSTRUCTIONS:"
echo "1. Use your teleop controls to drive around"
echo "2. Watch the map build in RViz"
echo "3. When done mapping, run: ./save_map.sh"
echo "4. Then Ctrl+C to stop this script"
echo ""
echo "Press Ctrl+C to stop all nodes."
echo "=========================================="

# Wait for processes and handle cleanup
trap "echo '';
      echo 'Stopping all nodes...';
      kill $RVIZ_PID $HARDWARE_PID $SLAM_PID $TELEOP_PID 2>/dev/null;
      sudo pkill -f qcar_hardware_interface;
      sudo pkill -f cmd_vel_to_qcar;
      echo 'Done. Remember to save your map if you haven'\''t yet!';
      exit" INT TERM

wait
