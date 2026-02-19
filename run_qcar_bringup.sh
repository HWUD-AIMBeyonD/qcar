#!/bin/bash

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

echo "Starting QCar Unified System..."
echo "========================================"

# 1. Robot Description (URDF)
if [ -f ~/qcar_ws/install/robot_description/share/robot_description/launch/simple_robot_launch.py ]; then
    echo "Starting robot_description (with RViz2)..."
    ros2 launch robot_description simple_robot_launch.py &
    RVIZ_PID=$!
    sleep 3
fi

# 2. Unified Hardware Interface (Sudo)
echo "Starting unified hardware interface..."
sudo -E PYTHONPATH=$PYTHONPATH LD_LIBRARY_PATH=$LD_LIBRARY_PATH DISPLAY=$DISPLAY bash -c '
    source /opt/ros/dashing/setup.bash
    source ~/qcar_ws/install/setup.bash
    python3 ~/qcar_ws/install/qcar_nav2_bringup/lib/qcar_nav2_bringup/qcar_hardware_interface --ros-args -p max_speed:=0.5 -p max_steering_angle:=0.5
' &
HARDWARE_PID=$!
sleep 2

# 3. Simple EKF (Sensor Fusion)
echo "Starting simple EKF (sensor fusion)..."
ros2 launch qcar_odom sensor_fusion.launch.py &
FUSION_PID=$!

echo "========================================"
echo "QCar System Running!"
echo "Nodes active:"
echo "  ✓ robot_description (URDF, TF)"
echo "  ✓ RViz2 visualization"
echo "  ✓ qcar_hardware_interface (LiDAR, odom_raw, imu, cmd_vel)"
echo "  ✓ simple_ekf (fuses odom_raw + imu -> /odometry/filtered, /path_fused)"
echo "========================================"
echo "Press Ctrl+C to stop all nodes."

# Wait for Ctrl+C
trap "echo ''; echo 'Stopping...';
      kill $RVIZ_PID $HARDWARE_PID $FUSION_PID 2>/dev/null;
      sudo pkill -f qcar_hardware_interface;
      sudo pkill -f simple_ekf;
      sleep 1;
      cleanup_hardware;
      exit" INT TERM

wait
