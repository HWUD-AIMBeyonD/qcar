#!/bin/bash
# ==============================================================
# QCar Mapping with OptiTrack Ground-Truth Odometry
# ==============================================================
# Pipeline:
#   OptiTrack (HTTP) -> http_odom_node -> /odom_opti + TF odom->base
#   qcar_hardware_interface            -> /scan + /cmd_vel (TF disabled)
#   robot_state_publisher              -> TF base->lidar, base->base_footprint
#   cartographer_node                  -> /map + TF map->odom
#
# No simple_ekf here -- OptiTrack replaces encoder+IMU fusion entirely.
# Use run_qcar_mapping.sh for the encoder/IMU fallback.
#
# Prerequisites:
#   - QCar connected to OptiTrack WiFi (192.168.0.x network)
#   - OptiTrack streaming at http://192.168.0.3:8000/QCar/pose
#   - colcon build completed
# ==============================================================

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
echo "Starting QCar OPTITRACK MAPPING System"
echo "=========================================="

# --- 1. Hardware interface (sudo, TF DISABLED -- OptiTrack owns odom->base) ---
echo "✓ Starting unified hardware interface (odom TF disabled)..."
sudo -E PYTHONPATH=$PYTHONPATH LD_LIBRARY_PATH=$LD_LIBRARY_PATH DISPLAY=$DISPLAY bash -c '
    source /opt/ros/dashing/setup.bash
    source ~/qcar_ws/install/setup.bash
    python3 ~/qcar_ws/install/qcar_nav2_bringup/lib/qcar_nav2_bringup/qcar_hardware_interface --ros-args -p max_speed:=0.5 -p max_steering_angle:=0.5 -p publish_odom_tf:=false
' &
HARDWARE_PID=$!

echo "  -> Waiting 10 seconds for Hardware/LIDAR to spin up..."
sleep 10

# --- 2. Robot description (URDF + TF + RViz) ---
echo "✓ Starting robot_description..."
ros2 launch robot_description simple_robot_launch.py &
RVIZ_PID=$!

sleep 5

# --- 3. OptiTrack HTTP odometry (provides /odom_opti + TF odom->base) ---
echo "✓ Starting OptiTrack HTTP odom node..."
ros2 run qcar_http_odom http_odom_node --ros-args \
    -p pose_url:='http://192.168.0.3:8000/QCar/pose' \
    -p rate_hz:=50.0 \
    -p odom_frame:='odom' \
    -p child_frame:='base' \
    -p publish_tf:=true &
OPTI_PID=$!

echo "  -> Waiting 5 seconds for TF tree to stabilize..."
sleep 5

# --- 4. Cartographer SLAM (reads /scan + /odom_opti + TF odom->base) ---
echo "✓ Starting Cartographer..."
ros2 launch qcar_nav2_bringup cartographer_optitrack.launch.py &
CARTO_PID=$!

echo "=========================================="
echo "QCar OptiTrack Mapping System Running!"
echo ""
echo "Action: Open a new terminal and run:"
echo "        ros2 run qcar_teleop manual_teleop"
echo ""
echo "When done mapping, save the map:"
echo "        bash save_map.sh"
echo "=========================================="

# Wait for Ctrl+C
trap "echo ''; echo 'Stopping...';
      kill $RVIZ_PID $CARTO_PID $OPTI_PID 2>/dev/null;
      sudo kill $HARDWARE_PID 2>/dev/null;
      sudo pkill -f qcar_hardware_interface;
      sudo pkill -f http_odom_node;
      sleep 1;
      cleanup_hardware;
      exit" INT TERM

wait
