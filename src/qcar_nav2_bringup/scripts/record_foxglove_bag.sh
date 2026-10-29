#!/usr/bin/env bash
#
# Full-stack rosbag recorder for viewing a QCar nav run in Foxglove.
#
# Unlike run_benchmark_hw.sh (4 topics, for KPIs) this records EVERY topic
# (`ros2 bag record -a`): camera, /imu, /scan, /pointcloud, /map, /plan,
# /local_plan, /lookahead_point, costmaps, /odom_opti, /cmd_vel, /tf,
# /tf_static, /rosout ... i.e. everything RViz shows and more.
#
# Prerequisites:
#   - Nav stack running: bash run_qcar_nav_optitrack.sh <controller>
#   - Camera node running (publishing sensor_msgs/Image or CompressedImage)
#   - rosbag2 installed (`ros2 bag` available)
#
# This script ONLY records (plus the tf_static relay, which republishes
# transforms that already exist). You send the nav goal yourself.
#
# Usage:
#   ./record_foxglove_bag.sh <controller> [label]
#   ./record_foxglove_bag.sh rpp
#   ./record_foxglove_bag.sh vector loop_a
#
# <controller> (rpp | stanley | vpc; vector is accepted as vpc) is checked
# against the running controller server, like run_benchmark_hw.sh. It picks
# the bag name:
#   benchmarks/foxglove/<controller>_navfn_<label|timestamp>/
#
# Stop with Enter or Ctrl+C -- both close the bag cleanly (metadata.yaml).
#
# Why the tf_static relay: Dashing's recorder subscribes VOLATILE, so latched
# /tf_static sent at startup never reaches it and Foxglove would have no
# static frames. tf_static_relay.py re-sends them every second. /map is
# already republished every 2 s by direct_map_publisher.
#

set -uo pipefail
set -m

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

PLANNER="navfn"

usage() {
    echo "Usage: $0 <controller> [label]"
    echo ""
    echo "  <controller>  rpp | stanley | vpc (vector = vpc) -- must match"
    echo "                the stack you launched. Only a label; it selects nothing."
    echo "  [label]       optional bag name suffix (default: timestamp)"
}

if [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ]; then
    usage
    exit 0
fi
if [ $# -lt 1 ] || [ $# -gt 2 ]; then
    usage
    exit 1
fi

CONTROLLER="$1"
LABEL="${2:-$(date +%Y%m%d_%H%M%S)}"

[ "$CONTROLLER" = "vector" ] && CONTROLLER="vpc"
case "$CONTROLLER" in
    rpp)     CONTROLLER_EXE="rpp_controller_server" ;;
    stanley) CONTROLLER_EXE="stanley_controller_server" ;;
    vpc)     CONTROLLER_EXE="vector_pursuit_controller_server" ;;
    *) echo "[ERROR] <controller> must be rpp, stanley or vpc (got '$CONTROLLER')"; exit 1 ;;
esac

REPO_ROOT="$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel 2>/dev/null)"
[ -z "$REPO_ROOT" ] && REPO_ROOT="$HOME/qcar_ws"
BAGS_DIR="${REPO_ROOT}/benchmarks/foxglove"
BAG_PATH="${BAGS_DIR}/${CONTROLLER}_${PLANNER}_${LABEL}"

mkdir -p "$BAGS_DIR"

if [ -e "$BAG_PATH" ]; then
    echo "[ERROR] $BAG_PATH already exists -- pick another label"
    exit 1
fi

if ! ros2 bag -h >/dev/null 2>&1; then
    echo "[ERROR] 'ros2 bag' is not available -- is rosbag2 installed and sourced?"
    exit 1
fi

# ---------- Check the running stack matches the label ----------
if ! pgrep -f "$CONTROLLER_EXE" >/dev/null; then
    echo "[ERROR] <controller> $CONTROLLER but $CONTROLLER_EXE is not running."
    echo "        Start the stack with: bash run_qcar_nav_optitrack.sh ${CONTROLLER/vpc/vector}"
    exit 1
fi
for other in rpp_controller_server stanley_controller_server vector_pursuit_controller_server; do
    if [ "$other" != "$CONTROLLER_EXE" ] && pgrep -f "$other" >/dev/null; then
        echo "[ERROR] <controller> $CONTROLLER but $other is running -- the label would be wrong."
        exit 1
    fi
done
if ! pgrep -f navfn_planner >/dev/null; then
    echo "[WARN] navfn_planner is not running -- /plan will be missing from the bag"
fi

# ---------- Warn about missing sensor topics (records anyway) ----------
TOPICS="$(ros2 topic list -t 2>/dev/null)"
for t in /scan /imu /map /plan /odom_opti; do
    echo "$TOPICS" | grep -q "^$t " || echo "[WARN] $t not advertised yet"
done
if ! echo "$TOPICS" | grep -qE "sensor_msgs/(msg/)?(Image|CompressedImage)"; then
    echo "[WARN] no Image/CompressedImage topic found -- is the camera node running?"
fi

RELAY_PID=""
BAG_PID=""

stop_all() {
    # SIGINT lets `ros2 bag record` close the bag and write metadata.yaml
    if [ -n "$BAG_PID" ]; then
        kill -INT "$BAG_PID" 2>/dev/null || true
        for _ in $(seq 1 10); do
            kill -0 "$BAG_PID" 2>/dev/null || break
            sleep 0.5
        done
        kill -TERM "$BAG_PID" 2>/dev/null || true
        wait "$BAG_PID" 2>/dev/null || true
        BAG_PID=""
    fi
    if [ -n "$RELAY_PID" ]; then
        kill -INT "$RELAY_PID" 2>/dev/null || true
        wait "$RELAY_PID" 2>/dev/null || true
        RELAY_PID=""
    fi
}

cleanup() {
    echo ""
    echo "Interrupted -- closing bag..."
    stop_all
    echo "Bag saved: $BAG_PATH"
    exit 130
}
trap cleanup INT TERM

echo "============================================"
echo " QCar Foxglove recorder (all topics)"
echo "  Controller:  $CONTROLLER"
echo "  Planner:     $PLANNER"
echo "  Bag:         $BAG_PATH"
echo "============================================"

python3 "$SCRIPT_DIR/tf_static_relay.py" &
RELAY_PID=$!
sleep 2  # let the relay collect the latched transforms

read -r -p "Place the robot and press Enter to start recording..."

ros2 bag record -a -o "$BAG_PATH" &
BAG_PID=$!
sleep 2

if ! kill -0 "$BAG_PID" 2>/dev/null; then
    echo "[ERROR] ros2 bag record exited immediately -- see its error above."
    stop_all
    exit 1
fi

echo ""
echo "  >>> RECORDING all topics. Send your nav goal now. <<<"
read -r -p "  Press Enter (or Ctrl+C) to stop..."

stop_all
echo "Bag saved: $BAG_PATH"
du -sh "$BAG_PATH" 2>/dev/null
