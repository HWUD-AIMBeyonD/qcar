#!/bin/bash
# ==============================================================
# QCar RealSense RGB camera -> /qcar/rgbd_color[/compressed]
# ==============================================================
# Runs alongside run_qcar_nav_optitrack.sh (separate terminal) so the camera
# feed ends up in bags from record_foxglove_bag.sh.
#
# Usage:
#   bash run_qcar_camera.sh                       # uses config/rgbd_camera.yaml
#   bash run_qcar_camera.sh /path/to/other.yaml   # different params
#
# Started under sudo like the hardware interface -- the Quanser Video3D
# driver needs it. CAMERA_SUDO=0 runs it as the current user instead.
#
# Parameters go through a YAML file (__params:=) because Dashing has no
# --ros-args -p. Edit src/qcar_nav2_bringup/config/rgbd_camera.yaml to change
# resolution / rate / JPEG / depth.
# ==============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NODE="$SCRIPT_DIR/src/qcar_nav2_bringup/scripts/rgbd_camera_node.py"
PARAMS="${1:-$SCRIPT_DIR/src/qcar_nav2_bringup/config/rgbd_camera.yaml}"

source /opt/ros/dashing/setup.bash
source ~/qcar_ws/install/setup.bash

if [ ! -f "$PARAMS" ]; then
    echo "[ERROR] params file not found: $PARAMS"
    exit 1
fi

echo "* Starting RealSense RGB node (params: $PARAMS)..."
if [ "${CAMERA_SUDO:-1}" = "1" ]; then
    sudo -E PYTHONPATH=$PYTHONPATH LD_LIBRARY_PATH=$LD_LIBRARY_PATH bash -c "
        source /opt/ros/dashing/setup.bash
        source ~/qcar_ws/install/setup.bash
        python3 '$NODE' __params:='$PARAMS'
    " &
else
    python3 "$NODE" __params:="$PARAMS" &
fi
CAM_PID=$!

trap "echo ''; echo 'Stopping camera...';
      sudo pkill -INT -f rgbd_camera_node.py 2>/dev/null;
      sleep 2;
      sudo pkill -f rgbd_camera_node.py 2>/dev/null;
      exit" INT TERM

wait $CAM_PID
