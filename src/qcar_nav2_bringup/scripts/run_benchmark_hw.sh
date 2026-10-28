#!/usr/bin/env bash
#
# Hardware benchmark recorder for the QCar controllers with navfn (OptiTrack).
#
# Prerequisites: nav stack must be running via run_qcar_nav_optitrack.sh <controller>
#   - rosbag2 installed (`ros2 bag` available)
#   - OptiTrack streaming, /odom_opti updating
#
# This script ONLY records. It never sends goals, cancels goals, or publishes
# anything -- you send the nav goal yourself (RViz, send_nav_goal, or
# ros2 action send_goal) while it records.
#
# Usage:
#   ./run_benchmark_hw.sh <controller> <runs>
#   ./run_benchmark_hw.sh stanley 5
#
# <controller> (rpp | stanley | vpc; vector is accepted as vpc) only sets the
# label in the CSV and the bag name -- it does NOT choose the controller. That
# is the argument to run_qcar_nav_optitrack.sh. Before the first run the
# script checks the running controller server and navfn_planner match the
# label and aborts if not.
# It also picks the output folder: benchmarks/<controller>_navfn/
#
# <runs> is the TOTAL number of runs wanted for the controller. Runs already
# on disk count: if run1..run4 exist, `./run_benchmark_hw.sh rpp 5` records
# run5 only, so an interrupted session just picks up where it stopped.
#
# What it does per trial:
#   1. Prompts you to place the robot at the start position
#   2. Starts rosbag recording (RECORD_TOPICS below)
#   3. You send the goal; press Enter once the car has finished
#      (recording also stops by itself after RECORD_TIMEOUT seconds)
#   4. Stops recording
#   5. Runs compute_kpis_hw.py on the bag (offline, reads the bag file only)
#
# Next to each bag a <bag>_extra/ folder (ros2 bag record refuses an existing
# dir, so it can't live inside the bag) holds run_info.txt and a copy + md5
# of the param YAML in use.
#
# Background jobs are started with job control on (set -m) so the recorder
# can be stopped with SIGINT. Without it bash makes async jobs ignore SIGINT
# and the bag would never close cleanly (no metadata.yaml).
#

set -uo pipefail
set -m

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---------- Configuration ----------
# Planner label written to the CSV -- navfn_planner in nav2_online.launch.py
PLANNER="navfn"

# Recording stops automatically after this many seconds if Enter isn't pressed
RECORD_TIMEOUT=120

# Topics to record
RECORD_TOPICS="/plan /odom_opti /cmd_vel /rosout"

# ---------- Argument parsing ----------
usage() {
    echo "Usage: $0 <controller> <runs>"
    echo ""
    echo "  <controller>  rpp | stanley | vpc (vector = vpc) -- must match"
    echo "                the stack you launched. Only a label; it selects nothing."
    echo "  <runs>        total runs wanted for this controller; existing"
    echo "                runN bags are kept and skipped"
}

if [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ]; then
    usage
    exit 0
fi
if [ $# -ne 2 ]; then
    usage
    exit 1
fi

CONTROLLER="$1"
RUNS="$2"

if ! [[ "$RUNS" =~ ^[1-9][0-9]*$ ]]; then
    echo "[ERROR] <runs> must be a positive integer (got '$RUNS')"
    exit 1
fi

# vector is the launch-file name for Vector Pursuit; the benchmark label is vpc
[ "$CONTROLLER" = "vector" ] && CONTROLLER="vpc"
case "$CONTROLLER" in
    rpp)     CONTROLLER_EXE="rpp_controller_server";            PARAM_FILE="nav2_params_rpp.yaml" ;;
    stanley) CONTROLLER_EXE="stanley_controller_server";        PARAM_FILE="nav2_params_stanley.yaml" ;;
    vpc)     CONTROLLER_EXE="vector_pursuit_controller_server"; PARAM_FILE="nav2_params_vector_pursuit.yaml" ;;
    *) echo "[ERROR] <controller> must be rpp, stanley or vpc (got '$CONTROLLER')"; exit 1 ;;
esac

# Results are grouped per controller/planner combo, under the repo root
# (the git checkout this script lives in, else ~/qcar_ws), so it does not
# matter which directory you start the script from:
#   benchmarks/<controller>_navfn/kpi_results_hw.csv
#   benchmarks/<controller>_navfn/rosbags_hw/<combo>_runN/
#   benchmarks/<controller>_navfn/rosbags_hw/<combo>_runN_extra/
REPO_ROOT="$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel 2>/dev/null)"
[ -z "$REPO_ROOT" ] && REPO_ROOT="$HOME/qcar_ws"
OUT_DIR="${REPO_ROOT}/benchmarks/${CONTROLLER}_${PLANNER}"
CSV_FILE="${OUT_DIR}/kpi_results_hw.csv"
BAGS_DIR="${OUT_DIR}/rosbags_hw"

mkdir -p "$BAGS_DIR"

# Resume after the highest run number already recorded for this combo
# (<combo>_runN, <combo>_runN_<timestamp> and their _extra dirs all count)
LAST_RUN=0
for d in "$BAGS_DIR/${CONTROLLER}_${PLANNER}_run"*; do
    [ -e "$d" ] || continue
    n="${d##*_run}"
    n="${n%%_*}"
    if [[ "$n" =~ ^[0-9]+$ ]] && [ "$n" -gt "$LAST_RUN" ]; then
        LAST_RUN="$n"
    fi
done
START_RUN=$((LAST_RUN + 1))
if [ "$START_RUN" -gt "$RUNS" ]; then
    echo "[INFO] ${CONTROLLER}_${PLANNER} already has $LAST_RUN run(s) in $BAGS_DIR"
    echo "       (<runs> $RUNS). Pass a larger <runs> to record more."
    exit 0
fi

echo "============================================"
echo " QCar Nav2 HARDWARE Benchmark (record only)"
echo "  Runs:            $START_RUN..$RUNS (found $LAST_RUN existing)"
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

# ---------- Check the running stack matches the labels ----------
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
    echo "[ERROR] planner is $PLANNER but navfn_planner is not running."
    exit 1
fi

# The param YAML nav2_online.launch.py loads (installed share/config copy)
PARAM_YAML=""
PKG_PREFIX="$(ros2 pkg prefix qcar_nav2_bringup 2>/dev/null)"
for candidate in "${PKG_PREFIX}/share/qcar_nav2_bringup/config/${PARAM_FILE}" \
                 "$HOME/qcar_ws/install/qcar_nav2_bringup/share/qcar_nav2_bringup/config/${PARAM_FILE}"; do
    if [ -f "$candidate" ]; then
        PARAM_YAML="$candidate"
        break
    fi
done
[ -z "$PARAM_YAML" ] && echo "[WARN] installed ${PARAM_FILE} not found -- it won't be copied per run"

# git repo for run_info.txt: wherever this script lives, else the workspace
GIT_DIR_FOR_INFO=""
for d in "$SCRIPT_DIR" "$PWD" "$HOME/qcar_ws" "$HOME/qcar_ws/src"; do
    if git -C "$d" rev-parse --show-toplevel >/dev/null 2>&1; then
        GIT_DIR_FOR_INFO="$d"
        break
    fi
done

BAG_PID=""

write_run_info() {
    local file="$1" run="$2"
    {
        echo "date:        $(date --iso-8601=seconds)"
        echo "controller:  $CONTROLLER"
        echo "planner:     $PLANNER"
        echo "run:         $run"
        echo "uname -r:    $(uname -r)"
        if [ -n "$GIT_DIR_FOR_INFO" ]; then
            echo "git HEAD:    $(git -C "$GIT_DIR_FOR_INFO" rev-parse HEAD 2>/dev/null || echo unknown)"
            echo "git status --porcelain:"
            git -C "$GIT_DIR_FOR_INFO" status --porcelain 2>/dev/null
        else
            echo "git HEAD:    unknown"
            echo "git status --porcelain: unknown"
        fi
    } > "$file"
}

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
for run in $(seq "$START_RUN" "$RUNS"); do
    combo="${CONTROLLER}_${PLANNER}_run${run}"
    bag_path="${BAGS_DIR}/${combo}"
    echo "--- $combo ---"

    # ros2 bag record refuses to overwrite an existing bag
    if [ -e "$bag_path" ] || [ -e "${bag_path}_extra" ]; then
        bag_path="${bag_path}_$(date +%Y%m%d_%H%M%S)"
        echo "  [WARN] Bag already exists, recording to $bag_path instead"
    fi

    echo "  Run $run of $RUNS"
    read -r -p "  Place robot at start position and press Enter to start recording run $run..."

    extra_dir="${bag_path}_extra"
    mkdir -p "$extra_dir"
    write_run_info "$extra_dir/run_info.txt" "$run"
    if [ -n "$PARAM_YAML" ]; then
        cp "$PARAM_YAML" "$extra_dir/"
        (cd "$extra_dir" && md5sum "$(basename "$PARAM_YAML")" > "$(basename "$PARAM_YAML").md5")
    fi

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
    echo "  Bag saved (extras in $extra_dir)"

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
