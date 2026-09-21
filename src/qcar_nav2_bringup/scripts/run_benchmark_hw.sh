#!/usr/bin/env bash
#
# Hardware benchmark recorder for the QCar RPP controller (OptiTrack).
#
# Prerequisites: nav stack must be running via run_qcar_nav_optitrack.sh rpp
#   - rosbag2 installed (`ros2 bag` available)
#   - OptiTrack streaming, /odom_opti updating
#
# This script ONLY records. It never sends goals, cancels goals, or publishes
# anything -- you send the nav goal yourself (RViz, send_nav_goal, or
# ros2 action send_goal) while it records.
#
# Usage:
#   ./run_benchmark_hw.sh --controller stanley
#
# --controller only sets the label in the CSV and the bag name -- it does NOT
# choose the controller. That is the argument to run_qcar_nav_optitrack.sh.
# It also picks the output folder: benchmarks/<controller>_<planner>/
#
# What it does per trial:
#   1. Prompts you to place the robot at the start position
#   2. Starts rosbag recording (/plan, /odom_opti, /cmd_vel)
#   3. You send the goal; press Enter once the car has finished
#      (recording also stops by itself after RECORD_TIMEOUT seconds)
#   4. Stops recording
#   5. Runs compute_kpis_hw.py on the bag (offline, reads the bag file only)
#
# Background jobs are started with job control on (set -m) so the recorder
# can be stopped with SIGINT. Without it bash makes async jobs ignore SIGINT
# and the bag would never close cleanly (no metadata.yaml).
#

set -uo pipefail
set -m

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---------- Configuration ----------
RUNS=5

# Labels written to the CSV -- must match what the nav stack is running
# (the arg to run_qcar_nav_optitrack.sh, navfn_planner in nav2_online.launch.py)
CONTROLLER="rpp"
PLANNER="navfn"

# Results are grouped per controller/planner combo:
#   benchmarks/<controller>_<planner>/kpi_results_hw.csv
#   benchmarks/<controller>_<planner>/rosbags_hw/<combo>_runN/
# Left empty here and filled in after arg parsing, so --controller feeds it.
OUT_DIR=""
CSV_FILE=""
BAGS_DIR=""

# Recording stops automatically after this many seconds if Enter isn't pressed
RECORD_TIMEOUT=120

# Topics to record
RECORD_TOPICS="/plan /odom_opti /cmd_vel"

# ---------- Argument parsing ----------
while [[ $# -gt 0 ]]; do
    case $1 in
        --runs)    RUNS="$2"; shift 2 ;;
        --csv)     CSV_FILE="$2"; shift 2 ;;
        --bags-dir) BAGS_DIR="$2"; shift 2 ;;
        --controller) CONTROLLER="$2"; shift 2 ;;
        --planner) PLANNER="$2"; shift 2 ;;
        --out-dir) OUT_DIR="$2"; shift 2 ;;
        -h|--help)
            echo "Usage: $0 [--controller NAME] [--planner NAME] [--runs N]"
            echo "          [--out-dir DIR] [--csv FILE] [--bags-dir DIR]"
            echo ""
            echo "  --controller  rpp | stanley | vector -- must match the stack"
            echo "                you launched. Only a label; it selects nothing."
            echo "  --planner     defaults to navfn"
            echo "  --out-dir     defaults to benchmarks/<controller>_<planner>"
            echo "  --csv/--bags-dir override the paths derived from --out-dir"
            exit 0
            ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

# Derive the output paths from the combo unless explicitly overridden
[ -z "$OUT_DIR" ]   && OUT_DIR="benchmarks/${CONTROLLER}_${PLANNER}"
[ -z "$CSV_FILE" ]  && CSV_FILE="${OUT_DIR}/kpi_results_hw.csv"
[ -z "$BAGS_DIR" ]  && BAGS_DIR="${OUT_DIR}/rosbags_hw"

mkdir -p "$BAGS_DIR"

echo "============================================"
echo " QCar Nav2 HARDWARE Benchmark (record only)"
echo "  Runs:            $RUNS"
echo "  Controller:      $CONTROLLER"
echo "  Planner:         $PLANNER"
echo "  Topics:          $RECORD_TOPICS"
echo "  Bags directory:  $BAGS_DIR"
echo "  CSV output:      $CSV_FILE"
echo "============================================"
echo ""

if ! ros2 bag -h >/dev/null 2>&1; then
    echo "[ERROR] 'ros2 bag' is not available -- is rosbag2 installed and sourced?"
    exit 1
fi

BAG_PID=""

stop_bag() {
    # SIGINT lets `ros2 bag record` close the bag and write metadata.yaml
    [ -z "$BAG_PID" ] && return
    kill -INT "$BAG_PID" 2>/dev/null || true
    for _ in $(seq 1 10); do
        kill -0 "$BAG_PID" 2>/dev/null || break
        sleep 0.5
    done
    kill -TERM "$BAG_PID" 2>/dev/null || true
    wait "$BAG_PID" 2>/dev/null || true
    BAG_PID=""
}

cleanup() {
    echo ""
    echo "Interrupted -- closing bag..."
    stop_bag
    exit 130
}
trap cleanup INT TERM

# ---------- Main loop ----------
for run in $(seq 1 "$RUNS"); do
    combo="${CONTROLLER}_${PLANNER}_run${run}"
    bag_path="${BAGS_DIR}/${combo}"
    echo "--- $combo ---"

    # ros2 bag record refuses to overwrite an existing bag
    if [ -e "$bag_path" ]; then
        bag_path="${bag_path}_$(date +%Y%m%d_%H%M%S)"
        echo "  [WARN] Bag already exists, recording to $bag_path instead"
    fi

    echo "  Run $run of $RUNS"
    read -r -p "  Place robot at start position and press Enter to start recording run $run..."

    # Start recording
    echo "  Recording bag: $bag_path"
    ros2 bag record -o "$bag_path" $RECORD_TOPICS &
    BAG_PID=$!
    sleep 2  # let recorder discover and subscribe to the topics

    if ! kill -0 "$BAG_PID" 2>/dev/null; then
        echo "  [ERROR] ros2 bag record exited immediately -- see its error above. Stopping."
        exit 1
    fi

    echo ""
    echo "  >>> RECORDING. Send your nav goal now. <<<"
    timed_out=false
    if ! read -r -t "$RECORD_TIMEOUT" -p "  Press Enter when the car has finished (auto-stop in ${RECORD_TIMEOUT}s)..."; then
        echo ""
        echo "  [WARN] No Enter after ${RECORD_TIMEOUT}s -- stopping recording"
        timed_out=true
    fi

    # Stop recording
    stop_bag
    echo "  Bag saved"

    if [ "$timed_out" = true ]; then
        read -r -p "  Recording timed out. Compute KPIs for this run anyway? [y/N] " answer
        if [[ ! "$answer" =~ ^[Yy]$ ]]; then
            echo "  Skipping KPIs for $combo (bag kept at $bag_path)"
            echo ""
            continue
        fi
    fi

    # Compute KPIs (offline -- reads the bag file only)
    echo "  Computing KPIs..."
    if ! python3 "$SCRIPT_DIR/compute_kpis_hw.py" "$bag_path" \
        --controller "$CONTROLLER" \
        --planner "$PLANNER" \
        --run "$run" \
        --csv "$CSV_FILE" \
        --odom-topic /odom_opti; then
        echo "  [WARN] KPI computation failed for $combo -- bag kept at $bag_path"
    fi

    echo ""
done

echo "============================================"
echo " Benchmark complete!"
echo " Results: $CSV_FILE"
echo "============================================"

