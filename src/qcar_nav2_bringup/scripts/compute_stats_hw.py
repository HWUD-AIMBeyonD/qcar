#!/usr/bin/env python3
"""Compute timing / resource statistics from a QCar hardware benchmark bag.

Companion to compute_kpis_hw.py: reads the same bag offline (rosbag2_py if
available, else sqlite3 + hand-written CDR decoding, so it runs on the car's
Dashing / Python 3.6 install) and appends one row per run to
benchmarks/<controller>_<planner>/compute_results_hw.csv.

Topics read:
  /controller_timing  Float64MultiArray [pose_lookup_ms, compute_ms, loop_period_ms, pose_age_ms]
  /planner_timing     Float64MultiArray [plan_ms, success, path_length_m, num_poses]
  /odom_opti_stats    Float64MultiArray [fetch_ms, ok]
  /system_stats       String (JSON from system_monitor.py)
  /plan, /cmd_vel     only to find the active window

Every statistic is computed twice: over the whole bag (suffix _all) and over
the active window from the first /plan to the last non-zero /cmd_vel
(suffix _active). Values of -1 in loop_period_ms / pose_age_ms mean "not
available" and are skipped. Missing data leaves the cell empty.

Usage:
    python3 compute_stats_hw.py <bag_path> --controller rpp --planner hybrid --run 1
                                [--csv FILE]
"""

import argparse
import csv
import json
import math
import os
import sqlite3
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from compute_kpis_hw import CdrReader, find_db3, normalize_type  # noqa: E402

FLOAT_ARRAY_TYPE = "std_msgs/msg/Float64MultiArray"
STRING_TYPE = "std_msgs/msg/String"
PATH_TYPE = "nav_msgs/msg/Path"
TWIST_TYPE = "geometry_msgs/msg/Twist"

CONTROLLER_TIMING_TOPIC = "/controller_timing"
PLANNER_TIMING_TOPIC = "/planner_timing"
OPTI_STATS_TOPIC = "/odom_opti_stats"
SYSTEM_STATS_TOPIC = "/system_stats"
PLAN_TOPIC = "/plan"
CMD_VEL_TOPIC = "/cmd_vel"

TOPIC_TYPES = {
    CONTROLLER_TIMING_TOPIC: FLOAT_ARRAY_TYPE,
    PLANNER_TIMING_TOPIC: FLOAT_ARRAY_TYPE,
    OPTI_STATS_TOPIC: FLOAT_ARRAY_TYPE,
    SYSTEM_STATS_TOPIC: STRING_TYPE,
    PLAN_TOPIC: PATH_TYPE,
    CMD_VEL_TOPIC: TWIST_TYPE,
}

# --controller label -> process label in system_monitor.py
CONTROLLER_PROCESSES = {
    "rpp": "rpp_controller_server",
    "stanley": "stanley_controller_server",
    "vpc": "vector_pursuit_controller_server",
}

# CSV column prefix -> system_monitor.py process label (controller filled in later)
PROCESS_COLUMNS = [
    ("planner", "hybrid_planner_server"),
    ("controller", None),
    ("global_costmap", "nav2_costmap_2d"),
    ("bt_navigator", "bt_navigator"),
]


# ---------------------------------------------------------------------------
# Bag reading
#
# Both readers return {topic: [(timestamp_ns, value), ...]} sorted by time,
# where value is a list of floats (Float64MultiArray), a str (String), None
# (Path -- only the timestamp is used) or (linear_x, angular_z) (Twist).
# ---------------------------------------------------------------------------

def read_bag_rosbag2_py(bag_path):
    """Read with rosbag2_py (Foxy and newer). Raises if unavailable/unreadable."""
    from rclpy.serialization import deserialize_message
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from std_msgs.msg import Float64MultiArray, String
    from geometry_msgs.msg import Twist

    reader = None
    last_error = None
    for storage_id in ("sqlite3", ""):
        try:
            reader = SequentialReader()
            reader.open(
                StorageOptions(uri=bag_path, storage_id=storage_id),
                ConverterOptions(
                    input_serialization_format="cdr",
                    output_serialization_format="cdr",
                ),
            )
            break
        except Exception as e:  # noqa: BLE001 -- rosbag2_py raises RuntimeError
            last_error = e
            reader = None
    if reader is None:
        raise RuntimeError("rosbag2_py could not open bag: {}".format(last_error))

    topic_types = {}
    for info in reader.get_all_topics_and_types():
        topic_types[info.name] = normalize_type(info.type)

    out = dict((t, []) for t in TOPIC_TYPES)
    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        if topic not in TOPIC_TYPES or topic_types.get(topic) != TOPIC_TYPES[topic]:
            continue
        msg_type = TOPIC_TYPES[topic]
        if msg_type == FLOAT_ARRAY_TYPE:
            value = list(deserialize_message(data, Float64MultiArray).data)
        elif msg_type == STRING_TYPE:
            value = deserialize_message(data, String).data
        elif msg_type == TWIST_TYPE:
            msg = deserialize_message(data, Twist)
            value = (msg.linear.x, msg.angular.z)
        else:
            value = None
        out[topic].append((timestamp_ns, value))
    return out


class StatsCdrReader(CdrReader):
    """CdrReader plus the float64 array the timing topics need."""

    def float64_sequence(self):
        return [self.float64() for _ in range(self.uint32())]


def decode_float64_multi_array(data):
    r = StatsCdrReader(data)
    for _ in range(r.uint32()):  # layout.dim[]
        r.string()   # label
        r.uint32()   # size
        r.uint32()   # stride
    r.uint32()       # layout.data_offset
    return r.float64_sequence()


def decode_string(data):
    return StatsCdrReader(data).string()


def decode_twist(data):
    r = StatsCdrReader(data)
    linear_x = r.float64()
    for _ in range(4):  # linear.y, linear.z, angular.x, angular.y
        r.float64()
    return linear_x, r.float64()


def read_bag_sqlite(bag_path):
    """Read the sqlite3 storage directly (Dashing, or bag without metadata)."""
    db_files = find_db3(bag_path)
    if not db_files:
        raise RuntimeError("no .db3 files found in '{}'".format(bag_path))

    decoders = {
        FLOAT_ARRAY_TYPE: decode_float64_multi_array,
        STRING_TYPE: decode_string,
        TWIST_TYPE: decode_twist,
        PATH_TYPE: lambda data: None,
    }
    out = dict((t, []) for t in TOPIC_TYPES)
    for db_file in db_files:
        conn = sqlite3.connect(db_file)
        try:
            wanted = {}
            for topic_id, name, type_name in conn.execute(
                    "SELECT id, name, type FROM topics"):
                if TOPIC_TYPES.get(name) == normalize_type(type_name):
                    wanted[topic_id] = name
            if not wanted:
                continue
            query = ("SELECT topic_id, timestamp, data FROM messages "
                     "WHERE topic_id IN ({}) ORDER BY timestamp").format(
                         ",".join(str(i) for i in wanted))
            for topic_id, timestamp_ns, data in conn.execute(query):
                topic = wanted[topic_id]
                try:
                    value = decoders[TOPIC_TYPES[topic]](data)
                except (struct.error, IndexError, ValueError) as e:
                    print("  [WARN] skipping undecodable {} message: {}".format(topic, e))
                    continue
                out[topic].append((timestamp_ns, value))
        finally:
            conn.close()

    for msgs in out.values():
        msgs.sort(key=lambda m: m[0])
    return out


def read_bag(bag_path):
    try:
        result = read_bag_rosbag2_py(bag_path)
        print("  Reader: rosbag2_py")
        return result
    except ImportError:
        print("  Reader: sqlite3 (rosbag2_py not available -- e.g. Dashing)")
    except Exception as e:  # noqa: BLE001
        print("  Reader: sqlite3 (rosbag2_py failed: {})".format(e))
    return read_bag_sqlite(bag_path)


# ---------------------------------------------------------------------------
# Statistics (no numpy on the car)
# ---------------------------------------------------------------------------

def mean(values):
    return sum(values) / len(values) if values else None


def std(values):
    """Population standard deviation."""
    if not values:
        return None
    m = mean(values)
    return math.sqrt(sum((v - m) ** 2 for v in values) / len(values))


def percentile(values, pct):
    """Linear interpolation between closest ranks (numpy's default)."""
    if not values:
        return None
    s = sorted(values)
    k = (len(s) - 1) * pct / 100.0
    lo = int(math.floor(k))
    hi = min(lo + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


def maximum(values):
    return max(values) if values else None


def active_window(bag):
    """(start_ns, end_ns) from the first /plan to the last non-zero /cmd_vel."""
    if not bag[PLAN_TOPIC]:
        return None
    start = bag[PLAN_TOPIC][0][0]
    nonzero = [t for t, (vx, wz) in bag[CMD_VEL_TOPIC] if vx != 0.0 or wz != 0.0]
    if not nonzero or nonzero[-1] < start:
        return None
    return start, nonzero[-1]


def in_window(msgs, window):
    if window is None:
        return []
    return [m for m in msgs if window[0] <= m[0] <= window[1]]


def column_values(msgs, index, skip_negative=False):
    values = []
    for _, data in msgs:
        if data is None or len(data) <= index:
            continue
        v = data[index]
        if skip_negative and v < 0.0:
            continue
        values.append(v)
    return values


def parse_system_stats(msgs):
    parsed = []
    for _, text in msgs:
        try:
            parsed.append(json.loads(text))
        except (TypeError, ValueError):
            continue
    return parsed


def stat_values(stats, getter):
    values = []
    for s in stats:
        try:
            v = getter(s)
        except (KeyError, TypeError, AttributeError):
            continue
        if v is not None:
            values.append(float(v))
    return values


def compute_stats(bag, window, controller_process):
    """{column_name_without_suffix: value} for the messages inside window."""
    def pick(topic):
        return bag[topic] if window == "all" else in_window(bag[topic], window)

    row = {}

    ctrl = pick(CONTROLLER_TIMING_TOPIC)
    compute = column_values(ctrl, 1)
    lookup = column_values(ctrl, 0)
    period = column_values(ctrl, 2, skip_negative=True)
    age = column_values(ctrl, 3, skip_negative=True)
    row["ctrl_compute_ms_mean"] = mean(compute)
    row["ctrl_compute_ms_p50"] = percentile(compute, 50)
    row["ctrl_compute_ms_p95"] = percentile(compute, 95)
    row["ctrl_compute_ms_p99"] = percentile(compute, 99)
    row["ctrl_compute_ms_max"] = maximum(compute)
    row["ctrl_pose_lookup_ms_mean"] = mean(lookup)
    row["ctrl_pose_lookup_ms_p95"] = percentile(lookup, 95)
    row["ctrl_loop_period_ms_mean"] = mean(period)
    row["ctrl_loop_period_ms_std"] = std(period)
    row["ctrl_loop_period_ms_p95"] = percentile(period, 95)
    period_mean = mean(period)
    row["ctrl_rate_hz"] = 1000.0 / period_mean if period_mean else None
    row["ctrl_pose_age_ms_mean"] = mean(age)
    row["ctrl_pose_age_ms_p95"] = percentile(age, 95)

    plans = pick(PLANNER_TIMING_TOPIC)
    plan_ms = column_values(plans, 0)
    row["plan_ms_each"] = ";".join("{:.1f}".format(v) for v in plan_ms)
    row["plan_ms_mean"] = mean(plan_ms)
    row["plan_ms_max"] = maximum(plan_ms)
    row["plan_count"] = len(plans)
    row["plan_success_count"] = sum(1 for v in column_values(plans, 1) if v >= 0.5)

    system = parse_system_stats(pick(SYSTEM_STATS_TOPIC))
    cpu_total = stat_values(system, lambda s: s["cpu"]["total"])
    row["cpu_total_pct_mean"] = mean(cpu_total)
    row["cpu_total_pct_max"] = maximum(cpu_total)
    for prefix, label in PROCESS_COLUMNS:
        label = label or controller_process
        cpu = stat_values(system, lambda s: s["procs"][label]["cpu_pct"])
        rss = stat_values(system, lambda s: s["procs"][label]["rss_mb"])
        row[prefix + "_cpu_pct_mean"] = mean(cpu)
        row[prefix + "_cpu_pct_max"] = maximum(cpu)
        row[prefix + "_rss_mb_max"] = maximum(rss)
    ram = stat_values(system, lambda s: s["ram"]["used_mb"])
    row["ram_used_mb_mean"] = mean(ram)
    row["ram_used_mb_max"] = maximum(ram)
    row["gpu_load_pct_mean"] = mean(stat_values(system, lambda s: s["gpu_load_pct"]))
    temps = []
    for s in system:
        temps.extend(stat_values(s.get("thermal") or [], lambda z: z["temp_c"]))
    row["temp_max_c"] = maximum(temps)

    opti = pick(OPTI_STATS_TOPIC)
    fetch = column_values(opti, 0)
    ok = column_values(opti, 1)
    row["opti_fetch_ms_mean"] = mean(fetch)
    row["opti_fetch_ms_p95"] = percentile(fetch, 95)
    row["opti_failure_rate"] = (
        sum(1 for v in ok if v < 0.5) / float(len(ok)) if ok else None)

    return row


def fmt(value):
    if value is None:
        return ""
    if isinstance(value, float):
        return "{:.3f}".format(value)
    return str(value)


def main():
    parser = argparse.ArgumentParser(
        description="Compute timing/resource stats from a hardware rosbag")
    parser.add_argument("bag_path", help="Path to the rosbag directory")
    parser.add_argument("--controller", default="rpp",
                        help="rpp | stanley | vpc (vector is accepted as vpc)")
    parser.add_argument("--planner", default="hybrid", help="Planner name for CSV")
    parser.add_argument("--run", type=int, default=1, help="Run number for CSV")
    parser.add_argument("--csv", default=None,
                        help="CSV output file (append mode); default "
                             "benchmarks/<controller>_<planner>/compute_results_hw.csv")
    args = parser.parse_args()

    controller = "vpc" if args.controller == "vector" else args.controller
    if controller not in CONTROLLER_PROCESSES:
        print("Error: --controller must be rpp, stanley or vpc (got '{}')".format(
            args.controller))
        sys.exit(1)
    csv_path = args.csv or os.path.join(
        "benchmarks", "{}_{}".format(controller, args.planner), "compute_results_hw.csv")

    if not os.path.exists(args.bag_path):
        print("Error: bag path '{}' not found".format(args.bag_path))
        sys.exit(1)

    print("Reading bag: {}".format(args.bag_path))
    bag = read_bag(args.bag_path)
    for topic in sorted(TOPIC_TYPES):
        print("  {:<20} {} messages".format(topic, len(bag[topic])))

    window = active_window(bag)
    if window is None:
        print("  [WARN] no active window (need a /plan and a later non-zero "
              "/cmd_vel) -- _active columns will be empty")
    else:
        print("  Active window: {:.2f} s".format((window[1] - window[0]) / 1e9))

    stats_all = compute_stats(bag, "all", CONTROLLER_PROCESSES[controller])
    if window is None:
        stats_active = dict((k, None) for k in stats_all)
    else:
        stats_active = compute_stats(bag, window, CONTROLLER_PROCESSES[controller])

    names = list(stats_all.keys())
    header = ["controller", "planner", "run", "bag"]
    values = [controller, args.planner, args.run, os.path.basename(
        os.path.normpath(args.bag_path))]
    for suffix, stats in (("_all", stats_all), ("_active", stats_active)):
        for name in names:
            header.append(name + suffix)
            values.append(fmt(stats[name]))

    print()
    print("=" * 60)
    for name in ("ctrl_compute_ms_mean", "ctrl_compute_ms_p95", "ctrl_rate_hz",
                 "ctrl_pose_age_ms_mean", "plan_ms_mean", "plan_count",
                 "cpu_total_pct_mean", "ram_used_mb_max", "temp_max_c",
                 "opti_fetch_ms_mean", "opti_failure_rate"):
        print("  {:<24} all={:>10}  active={:>10}".format(
            name, fmt(stats_all[name]), fmt(stats_active[name])))
    print("=" * 60)

    csv_dir = os.path.dirname(csv_path)
    if csv_dir and not os.path.isdir(csv_dir):
        os.makedirs(csv_dir)
    write_header = not os.path.exists(csv_path)
    with open(csv_path, "a", newline="") as f:
        writer = csv.writer(f)
        if write_header:
            writer.writerow(header)
        writer.writerow(values)
    print("\nResults appended to {}".format(csv_path))


if __name__ == "__main__":
    main()
