#!/bin/bash
# ==============================================================
# QCar Autonomous Navigation with OptiTrack Localization
# ==============================================================
# Pipeline:
#   qcar_hardware_interface        -> /scan, /cmd_vel (no odom TF)
#   http_odom_node (qcar_http_odom)-> /odom_opti + TF odom->base
#   robot_state_publisher          -> TF base->lidar, base->base_footprint
#   direct_map_publisher           -> /map (for costmaps)
#   static TF                      -> map->odom (identity, no AMCL needed)
#   nav2_online.launch.py          -> planner + controller + BT navigator
#
# No AMCL and no simple_ekf here -- OptiTrack is ground truth, so map == odom.
# Use run_qcar_bringup.sh for the encoder/IMU fallback.
#
# Usage:
#   bash run_qcar_nav_optitrack.sh              # defaults to RPP
#   bash run_qcar_nav_optitrack.sh stanley      # use Stanley controller
#   bash run_qcar_nav_optitrack.sh vector       # use Vector Pursuit
#
# Prerequisites:
#   - QCar connected to OptiTrack WiFi (192.168.0.x network)
#   - OptiTrack streaming at http://192.168.0.3:8000/QCar/pose
#   - Map saved at ~/qcar_ws/maps/new_map.yaml
#   - colcon build completed
# ==============================================================

# NOTE: no `set -e`. The hardware reset below routinely exits non-zero when a
# handle is already free, and under `set -e` that would abort the whole script.

CONTROLLER_TYPE="${1:-rpp}"

# map -> odom offset. The correct constant for new_map is baked into
# localization_optitrack.launch.py as the arg defaults; these env vars only
# override it, e.g. when running against a different map:
#   MAP_ODOM_X=1.2 MAP_ODOM_Y=-0.4 MAP_ODOM_YAW=0.3 bash run_qcar_nav_optitrack.sh
# MAP_ODOM_YAW is in RADIANS.
MAP_ODOM_ARGS=""
[ -n "$MAP_ODOM_X" ]   && MAP_ODOM_ARGS="$MAP_ODOM_ARGS map_odom_x:=$MAP_ODOM_X"
[ -n "$MAP_ODOM_Y" ]   && MAP_ODOM_ARGS="$MAP_ODOM_ARGS map_odom_y:=$MAP_ODOM_Y"
[ -n "$MAP_ODOM_YAW" ] && MAP_ODOM_ARGS="$MAP_ODOM_ARGS map_odom_yaw:=$MAP_ODOM_YAW"

source /opt/ros/dashing/setup.bash
source ~/qcar_ws/install/setup.bash

# --- CLEANUP FUNCTION ---
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
echo "Starting QCar OPTITRACK NAVIGATION System"
echo "Controller: ${CONTROLLER_TYPE}"
echo "=========================================="

# Release any QCar/LiDAR handles left locked by a previous run -- if these are
# still held, hardware init fails silently and you get no /scan and no motors.
echo "* Pre-flight hardware reset..."
sudo pkill -f qcar_hardware_interface 2>/dev/null
sudo pkill -f http_odom_node 2>/dev/null
sleep 1
cleanup_hardware
sleep 2

# --- 1. Hardware interface (sudo) ---
# NOTE: no -p publish_odom_tf:=false on purpose. publish_tf() in
# qcar_hardware_interface.py is never called (update_odometry is commented
# out), so the hardware interface publishes no odom->base TF either way, and
# a bool override on Dashing's CLI is a needless way to kill the node.
# Same reasoning as run_qcar_mapping_optitrack.sh.
echo "* Starting unified hardware interface..."
sudo -E PYTHONPATH=$PYTHONPATH LD_LIBRARY_PATH=$LD_LIBRARY_PATH DISPLAY=$DISPLAY bash -c '
    source /opt/ros/dashing/setup.bash
    source ~/qcar_ws/install/setup.bash
    python3 ~/qcar_ws/install/qcar_nav2_bringup/lib/qcar_nav2_bringup/qcar_hardware_interface --ros-args -p max_speed:=0.5 -p max_steering_angle:=0.5
' &
HARDWARE_PID=$!

echo "  -> Waiting 10 seconds for Hardware/LIDAR to spin up..."
sleep 10

# --- 2. Robot description (URDF + TF + RViz) ---
echo "* Starting robot_description + RViz..."
if [ -f ~/qcar_ws/install/robot_description/share/robot_description/launch/simple_robot_launch.py ]; then
    ros2 launch robot_description simple_robot_launch.py &
    RVIZ_PID=$!
fi

echo "  -> Waiting 20 SECONDS for TF tree/RViz to stabilize..."
for i in {20..1}; do
    echo -ne "     $i... \r"
    sleep 1
done
echo ""

# --- 3. OptiTrack HTTP odometry (/odom_opti + TF odom->base) ---
# yaw_offset_deg MUST match cartographer_optitrack.launch.py (90.0) -- the map
# was built in that frame, so a different offset here rotates the robot
# relative to the map by that difference.
# Started through the launch file, NOT `ros2 run ... --ros-args -p ...`:
# --ros-args/-p only exist from Eloquent on, so Dashing silently ignores them
# and the node falls back to its default yaw_offset_deg of 0.0.
echo "* Starting OptiTrack HTTP odom node..."
ros2 launch qcar_http_odom http_odom.launch.py \
    pose_url:='http://192.168.0.3:8000/QCar/pose' &
OPTI_PID=$!

echo "  -> Waiting 5 seconds for odom->base TF..."
sleep 5

# --- 4. Localization (map publisher + static identity map->odom TF) ---
echo "* Starting OptiTrack localization (map + static map->odom)..."
ros2 launch qcar_nav2_bringup localization_optitrack.launch.py ${MAP_ODOM_ARGS} &
LOC_PID=$!

sleep 3

# --- 5. Nav2 stack (costmap + planner + controller + BT navigator) ---
echo "* Starting Nav2 with controller: ${CONTROLLER_TYPE}..."
ros2 launch qcar_nav2_bringup nav2_online.launch.py controller_type:=${CONTROLLER_TYPE} &
NAV_PID=$!

echo "=========================================="
echo " QCar OptiTrack Navigation System Running!"
echo " Controller: ${CONTROLLER_TYPE}"
echo ""
echo " In RViz:"
echo "   1. Set nav goal (2D Nav Goal)"
echo "   2. Robot should navigate autonomously"
echo ""
echo " No initial pose needed -- OptiTrack provides"
echo " ground-truth localization (no AMCL)."
echo "=========================================="

# Wait for Ctrl+C
trap "echo ''; echo 'Stopping...';
      kill $RVIZ_PID $LOC_PID $NAV_PID $OPTI_PID 2>/dev/null;
      sudo kill $HARDWARE_PID 2>/dev/null;
      sudo pkill -f qcar_hardware_interface;
      sudo pkill -f http_odom_node;
      sleep 1;
      cleanup_hardware;
      exit" INT TERM

wait

