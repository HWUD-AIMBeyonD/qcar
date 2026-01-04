#!/bin/bash
# QCar Mapping Mode: Bringup + SLAM + Custom Teleop
# "Safe Mode" - Long delays to prevent TF race conditions

source /opt/ros/dashing/setup.bash
source ~/qcar_ws/install/setup.bash

# --- CLEANUP FUNCTION (Runs ONLY on exit) ---
function cleanup_hardware {
    echo "  -> Force-resetting QCar Hardware (Motors + LiDAR)..."
    sudo -E PYTHONPATH=$PYTHONPATH LD_LIBRARY_PATH=$LD_LIBRARY_PATH bash -c 'python3 -c "
import sys
sys.path.insert(0, \"/home/nvidia/Core Modules/Python\")
sys.path.insert(0, \"/home/nvidia/Core Modules/Python/Quanser\")

try:
    from Quanser.product_QCar import QCar
    car = QCar()
    car.terminate()
    print(\"     [+] Motors stopped.\")
except Exception as e:
    print(f\"     [-] Car reset failed: {e}\")

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
echo "Starting QCar MAPPING System (SAFE MODE)"
echo "=========================================="

# 1. Start Unified Hardware Interface (Prioritized)
echo "✓ Starting unified hardware interface..."
sudo -E PYTHONPATH=$PYTHONPATH LD_LIBRARY_PATH=$LD_LIBRARY_PATH DISPLAY=$DISPLAY bash -c '
    source /opt/ros/dashing/setup.bash
    source ~/qcar_ws/install/setup.bash
    python3 ~/qcar_ws/install/qcar_nav2_bringup/lib/qcar_nav2_bringup/qcar_hardware_interface --ros-args -p max_speed:=0.5 -p max_steering_angle:=0.5
' &
HARDWARE_PID=$!

echo "  -> Waiting 10 seconds for Hardware/LIDAR to spin up..."
sleep 10

# 2. Start robot_description (URDF + TF + RViz)
if [ -f ~/qcar_ws/install/robot_description/share/robot_description/launch/simple_robot_launch.py ]; then
    echo "✓ Starting robot_description..."
    ros2 launch robot_description simple_robot_launch.py &
    RVIZ_PID=$!
    
    echo "  -> Waiting 20 SECONDS for TF tree/RViz to stabilize..."
    # This gives plenty of time for /scan and /odom to appear before SLAM starts
    for i in {20..1}; do
        echo -ne "     $i... \r"
        sleep 1
    done
    echo ""
fi

# 3. Start SLAM Toolbox
echo "✓ Starting slam_toolbox..."
ros2 launch qcar_nav2_bringup slam_launch.launch.py &
SLAM_PID=$!

echo "=========================================="
echo "QCar Mapping System Running!"
echo "Action: Open a new terminal and run:"
echo "        ros2 run qcar_teleop manual_teleop"
echo "=========================================="

# Wait for Ctrl+C
trap "echo ''; echo 'Stopping...'; 
      kill $RVIZ_PID $SLAM_PID 2>/dev/null; 
      sudo kill $HARDWARE_PID 2>/dev/null;
      sudo pkill -f qcar_hardware_interface; 
      sleep 1;
      cleanup_hardware; 
      exit" INT TERM

wait
