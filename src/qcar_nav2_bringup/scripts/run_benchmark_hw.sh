#!/usr/bin/env bash
#
# Hardware benchmark runner for the QCar RPP controller (OptiTrack).
#
# Prerequisites: nav stack must be running via run_qcar_nav_optitrack.sh rpp
#   - All Nav2 lifecycle nodes active (/NavigateToPose must show up in
#     `ros2 action list`; if not, configure/activate them by hand first)
#   - OptiTrack streaming, /odom_opti updating
#
# Usage:
#   ./run_benchmark_hw.sh [--runs N] [--csv FILE] [--bags-dir DIR]
#
# What it does per trial:
#   1. Prompts you to place the robot at the start position
#   2. Starts rosbag recording (/plan, /odom_opti, /cmd_vel)
#   3. Sends a nav goal via ros2 action
#   4. Waits for navigation to complete (or timeout)
#   5. Stops recording
#   6. Runs compute_kpis_hw.py on the bag
#
# Differences from the simulation run_benchmark.sh:
#   - No Gazebo pose reset: you reposition the car by hand before each run.
#   - Dashing action name is /NavigateToPose (not /navigate_to_pose).
#   - Background jobs are started with job control on (set -m) so they can be
#     stopped with SIGINT. Without it bash makes async jobs ignore SIGINT, so
#     the bag would never close cleanly and a timed-out goal would never be
#     cancelled -- the car would keep driving.
#   - No `set -e`: one bad trial (e.g. no /plan recorded) shouldn't abort the
#     remaining runs.
#

set -uo pipefail
set -m

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---------- Configuration ----------
RUNS=5
CSV_FILE="kpi_results_hw.csv"
BAGS_DIR="rosbags_hw"

# Controllers to test -- hardware stack only supports one per launch, and
# run_qcar_nav_optitrack.sh must have been started with the same one.
CONTROLLERS=("rpp")

# Planners to test (label only -- written to the CSV). The hardware stack
# runs navfn_planner (nav2_online.launch.py).
PLANNERS=("navfn")

# Navigation goal (x, y, yaw) in map frame, on new_map. Converted from the
# /odom_opti pose (1.7124, -0.7898, yaw 93.44 deg) through the map->odom
# static transform in localization_optitrack.launch.py (2.242934, 1.773473,
# 3.094191). Recompute if that transform changes.
GOAL_X=0.5699
GOAL_Y=2.6435
GOAL_YAW=-1.558230   # radians (-89.28 deg)

# Timeout in seconds for each navigation trial
NAV_TIMEOUT=120

# Topics to record
RECORD_TOPICS="/plan /odom_opti /cmd_vel"

# ---------- Argument parsing ----------
while [[ $# -gt 0 ]]; do
    case $1 in
        --runs)    RUNS="$2"; shift 2 ;;
        --csv)     CSV_FILE="$2"; shift 2 ;;
        --bags-dir) BAGS_DIR="$2"; shift 2 ;;
        -h|--help)
            echo "Usage: $0 [--runs N] [--csv FILE] [--bags-dir DIR]"
            exit 0
            ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

mkdir -p "$BAGS_DIR"

echo "============================================"
echo " QCar Nav2 HARDWARE Benchmark"
echo "  Runs per combo:  $RUNS"
echo "  Controllers:     ${CONTROLLERS[*]}"
echo "  Planners:        ${PLANNERS[*]}"
echo "  Goal (map):      ($GOAL_X, $GOAL_Y, yaw=$GOAL_YAW)"
echo "  Bags directory:  $BAGS_DIR"
echo "  CSV output:      $CSV_FILE"
echo "============================================"
echo ""

# Fail fast if Nav2 isn't accepting goals (lifecycle nodes not active)
if ! ros2 action list 2>/dev/null | grep -q "^/NavigateToPose$"; then
    echo "[ERROR] /NavigateToPose action server not found."
    echo "        Is run_qcar_nav_optitrack.sh rpp running, and is bt_navigator active?"
    echo "        Check: ros2 lifecycle get /bt_navigator"
    exit 1
fi

NAV_PID=""
BAG_PID=""

send_nav_goal() {
    # Send NavigateToPose action goal
    ros2 action send_goal /NavigateToPose nav2_msgs/action/NavigateToPose \
        "{pose: {header: {frame_id: 'map'}, pose: {position: {x: $GOAL_X, y: $GOAL_Y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: $(python3 -c "import math; print(math.sin($GOAL_YAW/2))"), w: $(python3 -c "import math; print(math.cos($GOAL_YAW/2))")}}}}" \
        2>&1 &
    NAV_PID=$!
}

stop_job() {
    # SIGINT lets `ros2 action send_goal` cancel its goal and lets
    # `ros2 bag record` close the bag and write metadata.yaml.
    local pid=$1
    [ -z "$pid" ] && return
    kill -INT "$pid" 2>/dev/null || true
    for _ in $(seq 1 10); do
        kill -0 "$pid" 2>/dev/null || break
        sleep 0.5
    done
    kill -TERM "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
}

wait_for_nav() {
    local timeout=$1
    local elapsed=0
    while kill -0 "$NAV_PID" 2>/dev/null; do
        sleep 1
        elapsed=$((elapsed + 1))
        if [ "$elapsed" -ge "$timeout" ]; then
            echo "  [WARN] Navigation timed out after ${timeout}s -- cancelling goal"
            stop_job "$NAV_PID"
            return 1
        fi
    done
    wait "$NAV_PID" 2>/dev/null || true
    return 0
}

cleanup() {
    echo ""
    echo "Interrupted -- cancelling goal and closing bag..."
    stop_job "$NAV_PID"
    stop_job "$BAG_PID"
    exit 130
}
trap cleanup INT TERM

# ---------- Main loop ----------
for controller in "${CONTROLLERS[@]}"; do
    for planner in "${PLANNERS[@]}"; do
        for run in $(seq 1 "$RUNS"); do
            combo="${controller}_${planner}_run${run}"
            bag_path="${BAGS_DIR}/${combo}"
            echo "--- $combo ---"

            # ros2 bag record refuses to overwrite an existing bag
            if [ -e "$bag_path" ]; then
                bag_path="${bag_path}_$(date +%Y%m%d_%H%M%S)"
                echo "  [WARN] Bag already exists, recording to $bag_path instead"
            fi

            # Manual reset -- no Gazebo on hardware
            echo "  Run $run of $RUNS"
            read -r -p "  Place robot at start position and press Enter to begin run $run..."

            # Start recording
            echo "  Recording bag: $bag_path"
            ros2 bag record -o "$bag_path" $RECORD_TOPICS &
            BAG_PID=$!
            sleep 2  # let recorder discover and subscribe to the topics

            # Send goal
            echo "  Sending nav goal..."
            send_nav_goal

            # Wait for completion
            if wait_for_nav "$NAV_TIMEOUT"; then
                echo "  Navigation completed"
            fi
            NAV_PID=""

            # Stop recording
            sleep 1
            stop_job "$BAG_PID"
            BAG_PID=""
            echo "  Bag saved"

            # Compute KPIs
            echo "  Computing KPIs..."
            if ! python3 "$SCRIPT_DIR/compute_kpis_hw.py" "$bag_path" \
                --controller "$controller" \
                --planner "$planner" \
                --run "$run" \
                --csv "$CSV_FILE" \
                --odom-topic /odom_opti; then
                echo "  [WARN] KPI computation failed for $combo -- bag kept at $bag_path"
            fi

            echo ""
        done
    done
done

echo "============================================"
echo " Benchmark complete!"
echo " Results: $CSV_FILE"
echo "============================================"
