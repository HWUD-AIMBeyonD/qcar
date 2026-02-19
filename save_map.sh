#!/bin/bash
# Save the current SLAM map
# Run this in a NEW TERMINAL while run_qcar_mapping.sh is still running.

source /opt/ros/dashing/setup.bash
source ~/qcar_ws/install/setup.bash

# Get timestamp for unique filename
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
MAP_NAME="qcar_map_${TIMESTAMP}"

echo "=========================================="
echo "Saving map: ${MAP_NAME}"
echo "=========================================="

# 1. Save map using nav2_map_server (Dashing version: 'map_saver', not 'map_saver_cli')
cd ~/qcar_ws/maps
echo "Outputting to $(pwd)/${MAP_NAME}..."

# Try the Dashing executable name first
if [ -f /opt/ros/dashing/lib/nav2_map_server/map_saver ]; then
    ros2 run nav2_map_server map_saver -f ${MAP_NAME} --ros-args -p save_map_timeout:=10000
else
    # Fallback just in case
    ros2 run nav2_map_server map_saver_cli -f ${MAP_NAME} --ros-args -p save_map_timeout:=10000
fi

# 2. Serialize SLAM Toolbox map (Optional)
# Only works if SLAM Toolbox is currently running
echo ""
echo "Triggering slam_toolbox serialization..."
if ros2 service list | grep -q "/slam_toolbox/serialize_map"; then
    ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '${HOME}/qcar_ws/maps/${MAP_NAME}_slam'}"
else
    echo "SKIPPING: SLAM Toolbox service not found. Is the mapping node running?"
fi

echo "=========================================="
echo "Map saved successfully!"
echo "=========================================="
echo "Files created:"
echo "  • ${MAP_NAME}.pgm"
echo "  • ${MAP_NAME}.yaml"
echo ""
# echo "Updating symlink..."
# ln -sf ${MAP_NAME}.yaml latest_map.yaml
# ln -sf ${MAP_NAME}.pgm latest_map.pgm
# echo "  • latest_map.yaml -> ${MAP_NAME}.yaml"
echo "=========================================="

