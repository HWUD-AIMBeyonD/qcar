#!/bin/bash
# QCar Mapping Mode: Bringup + SLAM + Custom Teleop

source /opt/ros/dashing/setup.bash
source ~/qcar_ws/install/setup.bash

# --- CLEANUP FUNCTION (Runs only on exit) ---
function cleanup_hardware {
    echo "  -> Force-resetting QCar Hardware (Motors + LiDAR)..."
    sudo -E PYTHONPATH=$PYTHONPATH LD_LIBRARY_PATH=$LD_LIBRARY_PATH bash -c 'python3 -c "
import sys
sys.path.insert(0, \"/home/nvidia/Core Modules/Python\")
sys.path.insert(0, \"/home/nvidia/Core Modules/Python/Quanser\")

# 1. Reset Car (Motors)
try:
    from Quanser.product_QCar import QCar
    car = QCar()
    car.terminate()
    print(\"     [+] Motors stopped.\")
except Exception as e:
    print(f\"     [-] Car reset failed: {e}\")

# 2. Reset LiDAR
try:
    try:
        from pal.utilities.lidar import Lidar
    except:
        from hal.utilities.lidar import Lidar
    lidar = Lidar()
    lidar.terminate()
    print(\"     [+] LiDAR stopped.\")
except Exception as e:
    print(f\"     [-] LiDAR reset failed: {e}\")
"'
}

echo "=========================================="
echo "Starting QCar MAPPING System..."
echo "=========================================="

# 1. Start robot_description
if [ -f ~/qcar_ws/install/robot_description/share/robot_description/launch/simple_robot_launch.py ]; then
    echo "✓ Starting robot_description..."
    ros2 launch robot_description simple_robot_launch.py &
    RVIZ_PID=$!
    sleep 3
fi

# 2. Start Unified Hardware Interface
echo "✓ Starting unified hardware interface..."
sudo -E PYTHONPATH=$PYTHONPATH LD_LIBRARY_PATH=$LD_LIBRARY_PATH DISPLAY=$DISPLAY bash -c '
    source /opt/ros/dashing/setup.bash
    source ~/qcar_ws/install/setup.bash
    python3 ~/qcar_ws/install/qcar_nav2_bringup/lib/qcar_nav2_bringup/qcar_hardware_interface --ros-args -p max_speed:=0.5 -p max_steering_angle:=0.5
' &
HARDWARE_PID=$!
sleep 2

# 3. Start SLAM Toolbox
echo "✓ Starting slam_toolbox..."
ros2 launch qcar_nav2_bringup slam_launch.launch.py &
SLAM_PID=$!
sleep 3

# 4. Start YOUR Custom Teleop Node (WASD)
#echo "✓ Starting Manual Teleop (WASD)..."
#gnome-terminal --title="QCar Teleop" -- bash -c "source /opt/ros/dashing/setup.bash; source ~/qcar_ws/install/setup.bash; ros2 run qcar_teleop manual_teleop; exec bash" &

#echo "=========================================="
#echo "QCar Mapping System Running!"
#echo "Check the 'QCar Teleop' terminal window to drive."
#echo "=========================================="

# Wait for processes
# CLEANUP runs here on exit
trap "echo ''; echo 'Stopping...'; 
      kill $RVIZ_PID $HARDWARE_PID $SLAM_PID; 
      sudo pkill -f qcar_hardware_interface; 
      sleep 1;
      cleanup_hardware; 
      exit" INT TERM

wait
