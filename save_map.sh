#!/bin/bash
# Save the current SLAM map
# Run this while slam_toolbox is still running (before stopping mapping script)

# Get timestamp for unique filename
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
MAP_NAME="qcar_map_${TIMESTAMP}"

echo "=========================================="
echo "Saving map: ${MAP_NAME}"
echo "=========================================="

# Save map using map_saver (for nav2)
cd ~/qcar_ws/maps
ros2 run nav2_map_server map_saver_cli -f ${MAP_NAME} --ros-args -p save_map_timeout:=10000

# Also save serialized map from slam_toolbox (optional, for continuing SLAM later)
echo ""
echo "Triggering slam_toolbox serialization..."
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '${HOME}/qcar_ws/maps/${MAP_NAME}_slam'}"

echo "=========================================="
echo "Map saved successfully!"
echo "=========================================="
echo "Files created:"
echo "  • ${MAP_NAME}.pgm (map image)"
echo "  • ${MAP_NAME}.yaml (map metadata)"
echo "  • ${MAP_NAME}_slam.posegraph (SLAM data)"
echo ""
echo "Latest map symlink:"
ln -sf ${MAP_NAME}.yaml latest_map.yaml
ln -sf ${MAP_NAME}.pgm latest_map.pgm
echo "  • latest_map.yaml -> ${MAP_NAME}.yaml"
echo "  • latest_map.pgm -> ${MAP_NAME}.pgm"
echo "=========================================="
