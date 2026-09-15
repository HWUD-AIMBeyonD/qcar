#!/usr/bin/env python3
"""Compute navigation KPIs from a ROS 2 bag file recorded on the QCar hardware.

Hardware adaptation of the simulation compute_kpis.py. The six KPIs, their
math and the CSV format are unchanged. Differences:

  * Odometry comes from OptiTrack (/odom_opti by default, --odom-topic).
  * The global plan topic is configurable (--plan-topic). Dashing's
    navfn_planner publishes nav_msgs/Path on /plan.
  * /plan is in the 'map' frame but /odom_opti is in 'odom'. The static
    map->odom transform (localization_optitrack.launch.py) is applied to the
    odometry before computing CTE; without it the CTE would just measure that
    ~2.9 m offset. Override with --map-odom-x/y/yaw if the launch defaults change.
  * Works on the car (Dashing, Python 3.6) as well as on a newer ROS 2 install.
    Dashing has neither rosbag2_py nor rclpy.serialization, so when those are
    missing (or can't open the bag) the .db3 file is read directly with
    sqlite3 and the three message types are decoded from CDR by hand. That
    path also works when metadata.yaml is missing (recorder killed hard).

Usage:
    python3 compute_kpis_hw.py <bag_path> [--controller NAME] [--run NUM] [--csv FILE]
                               [--odom-topic TOPIC] [--plan-topic TOPIC]

Example:
    python3 compute_kpis_hw.py rosbags_hw/rpp_navfn_run1 --controller rpp --run 1
"""

import argparse
import csv
import glob
import math
import os
import sqlite3
import struct
import sys

# map -> odom static transform, i.e. the pose of 'odom' in 'map'. Must match
# the defaults in localization_optitrack.launch.py.
DEFAULT_MAP_ODOM_X = 2.242934
DEFAULT_MAP_ODOM_Y = 1.773473
DEFAULT_MAP_ODOM_YAW = 3.094191

PATH_TYPE = "nav_msgs/msg/Path"
ODOM_TYPE = "nav_msgs/msg/Odometry"
TWIST_TYPE = "geometry_msgs/msg/Twist"


def normalize_type(type_name):
    """'nav_msgs/Path' -> 'nav_msgs/msg/Path' (older bags may omit /msg/)."""
    parts = type_name.split("/")
    if len(parts) == 2:
        return parts[0] + "/msg/" + parts[1]
    return type_name


# ---------------------------------------------------------------------------
# Bag reading
#
# Every reader returns the same thing:
#   plans:    [(timestamp_ns, frame_id, [(x, y), ...]), ...]
#   odoms:    [(timestamp_ns, x, y), ...]
#   cmd_vels: [(timestamp_ns, linear_x, angular_z), ...]
#   odom_frame: frame_id of the first odometry message ('' if none)
# ---------------------------------------------------------------------------

def read_bag_rosbag2_py(bag_path, plan_topic, odom_topic, cmd_vel_topic):
    """Read with rosbag2_py (Foxy and newer). Raises if unavailable/unreadable."""
    from rclpy.serialization import deserialize_message
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from nav_msgs.msg import Odometry, Path
    from geometry_msgs.msg import Twist

    type_map = {
        PATH_TYPE: Path,
        ODOM_TYPE: Odometry,
        TWIST_TYPE: Twist,
    }

    # Dashing bags are sqlite3. Name it explicitly first; fall back to ""
    # (auto-detect) for installs where the explicit id is rejected.
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

    plans = []
    odoms = []
    cmd_vels = []
    odom_frame = ""

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        msg_type = topic_types.get(topic)

        if topic == plan_topic and msg_type in type_map:
            msg = deserialize_message(data, type_map[msg_type])
            poses = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            plans.append((timestamp_ns, msg.header.frame_id, poses))

        elif topic == odom_topic and msg_type in type_map:
            msg = deserialize_message(data, type_map[msg_type])
            if not odom_frame:
                odom_frame = msg.header.frame_id
            odoms.append((timestamp_ns, msg.pose.pose.position.x,
                          msg.pose.pose.position.y))

        elif topic == cmd_vel_topic and msg_type in type_map:
            msg = deserialize_message(data, type_map[msg_type])
            cmd_vels.append((timestamp_ns, msg.linear.x, msg.angular.z))

    return plans, odoms, cmd_vels, odom_frame


class CdrReader(object):
    """Minimal CDR decoder for the handful of primitives the KPIs need."""

    def __init__(self, data):
        self.data = bytes(data)
        # 4-byte encapsulation header: 0x00 0x01 = little endian, 0x00 0x00 = big
        self.endian = "<" if self.data[1] == 1 else ">"
        self.offset = 4

    def _align(self, size):
        # Alignment is relative to the start of the payload (after the header)
        rel = self.offset - 4
        self.offset += (-rel) % size

    def _unpack(self, fmt, size):
        self._align(size)
        value = struct.unpack_from(self.endian + fmt, self.data, self.offset)[0]
        self.offset += size
        return value

    def uint32(self):
        return self._unpack("I", 4)

    def int32(self):
        return self._unpack("i", 4)

    def float64(self):
        return self._unpack("d", 8)

    def string(self):
        length = self.uint32()  # includes the trailing NUL
        raw = self.data[self.offset:self.offset + length]
        self.offset += length
        return raw.rstrip(b"\x00").decode("utf-8", "replace")

    def header(self):
        self.int32()   # stamp.sec
        self.uint32()  # stamp.nanosec
        return self.string()

    def point_xy(self):
        """Read a geometry_msgs/Pose; return (x, y)."""
        x = self.float64()
        y = self.float64()
        for _ in range(5):  # position.z + orientation x, y, z, w
            self.float64()
        return x, y


def decode_path(data):
    r = CdrReader(data)
    frame_id = r.header()
    poses = []
    for _ in range(r.uint32()):
        r.header()  # per-pose header
        poses.append(r.point_xy())
    return frame_id, poses


def decode_odometry(data):
    r = CdrReader(data)
    frame_id = r.header()
    r.string()  # child_frame_id
    x, y = r.point_xy()
    return frame_id, x, y


def decode_twist(data):
    r = CdrReader(data)
    linear_x = r.float64()
    r.float64()  # linear.y
    r.float64()  # linear.z
    r.float64()  # angular.x
    r.float64()  # angular.y
    angular_z = r.float64()
    return linear_x, angular_z


def find_db3(bag_path):
    if os.path.isfile(bag_path) and bag_path.endswith(".db3"):
        return [bag_path]
    return sorted(glob.glob(os.path.join(bag_path, "*.db3")))


def read_bag_sqlite(bag_path, plan_topic, odom_topic, cmd_vel_topic):
    """Read the sqlite3 storage directly (Dashing, or bag without metadata)."""
    db_files = find_db3(bag_path)
    if not db_files:
        raise RuntimeError("no .db3 files found in '{}'".format(bag_path))

    plans = []
    odoms = []
    cmd_vels = []
    odom_frame = ""

    for db_file in db_files:
        conn = sqlite3.connect(db_file)
        try:
            topics = {}
            for topic_id, name, type_name in conn.execute(
                    "SELECT id, name, type FROM topics"):
                topics[topic_id] = (name, normalize_type(type_name))

            wanted = {}
            for topic_id, (name, type_name) in topics.items():
                if name == plan_topic and type_name == PATH_TYPE:
                    wanted[topic_id] = "plan"
                elif name == odom_topic and type_name == ODOM_TYPE:
                    wanted[topic_id] = "odom"
                elif name == cmd_vel_topic and type_name == TWIST_TYPE:
                    wanted[topic_id] = "cmd_vel"
            if not wanted:
                continue

            query = ("SELECT topic_id, timestamp, data FROM messages "
                     "WHERE topic_id IN ({}) ORDER BY timestamp").format(
                         ",".join(str(i) for i in wanted))
            for topic_id, timestamp_ns, data in conn.execute(query):
                kind = wanted[topic_id]
                if kind == "plan":
                    frame_id, poses = decode_path(data)
                    plans.append((timestamp_ns, frame_id, poses))
                elif kind == "odom":
                    frame_id, x, y = decode_odometry(data)
                    if not odom_frame:
                        odom_frame = frame_id
                    odoms.append((timestamp_ns, x, y))
                else:
                    linear_x, angular_z = decode_twist(data)
                    cmd_vels.append((timestamp_ns, linear_x, angular_z))
        finally:
            conn.close()

    # Split bags: keep everything in time order across files
    plans.sort(key=lambda m: m[0])
    odoms.sort(key=lambda m: m[0])
    cmd_vels.sort(key=lambda m: m[0])
    return plans, odoms, cmd_vels, odom_frame


def read_bag(bag_path, plan_topic, odom_topic, cmd_vel_topic="/cmd_vel"):
    """Read plan, odom, and cmd_vel messages, using whichever reader works."""
    try:
        result = read_bag_rosbag2_py(bag_path, plan_topic, odom_topic, cmd_vel_topic)
        print("  Reader: rosbag2_py")
        return result
    except ImportError:
        print("  Reader: sqlite3 (rosbag2_py not available -- e.g. Dashing)")
    except Exception as e:  # noqa: BLE001
        print("  Reader: sqlite3 (rosbag2_py failed: {})".format(e))
    return read_bag_sqlite(bag_path, plan_topic, odom_topic, cmd_vel_topic)


def odom_to_map(odoms, tx, ty, yaw):
    """Express odometry points in the map frame: p_map = R(yaw) * p_odom + t."""
    c = math.cos(yaw)
    s = math.sin(yaw)
    return [(t, c * x - s * y + tx, s * x + c * y + ty) for t, x, y in odoms]


# ---------------------------------------------------------------------------
# KPIs (identical to the simulation version)
# ---------------------------------------------------------------------------

def point_to_segment_dist(px, py, ax, ay, bx, by):
    """Perpendicular distance from point (px,py) to segment (ax,ay)-(bx,by)."""
    dx = bx - ax
    dy = by - ay
    len_sq = dx * dx + dy * dy
    if len_sq < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / len_sq))
    proj_x = ax + t * dx
    proj_y = ay + t * dy
    return math.hypot(px - proj_x, py - proj_y)


def compute_cte(odoms, plan_poses):
    """Cross-track error: distance from each odom point to nearest plan segment."""
    if len(plan_poses) < 2 or not odoms:
        return 0.0, 0.0

    segments = []
    for i in range(len(plan_poses) - 1):
        segments.append((
            plan_poses[i][0], plan_poses[i][1],
            plan_poses[i + 1][0], plan_poses[i + 1][1],
        ))

    cte_values = []
    for _, ox, oy in odoms:
        min_dist = float("inf")
        for ax, ay, bx, by in segments:
            d = point_to_segment_dist(ox, oy, ax, ay, bx, by)
            if d < min_dist:
                min_dist = d
        cte_values.append(min_dist)

    rmse = math.sqrt(sum(v * v for v in cte_values) / len(cte_values))
    cte_max = max(cte_values)
    return rmse, cte_max


def compute_mission_time(cmd_vels):
    """Time between first and last cmd_vel where car is moving."""
    moving_times = [t for t, vx, _ in cmd_vels if abs(vx) > 0.01]
    if len(moving_times) < 2:
        return 0.0
    return (moving_times[-1] - moving_times[0]) / 1e9


def compute_accel_rms(cmd_vels):
    """RMS of linear and angular acceleration from cmd_vel differentiation."""
    if len(cmd_vels) < 2:
        return 0.0, 0.0

    lin_accels = []
    ang_accels = []
    for i in range(1, len(cmd_vels)):
        dt = (cmd_vels[i][0] - cmd_vels[i - 1][0]) / 1e9
        if dt < 1e-6:
            continue
        lin_accels.append((cmd_vels[i][1] - cmd_vels[i - 1][1]) / dt)
        ang_accels.append((cmd_vels[i][2] - cmd_vels[i - 1][2]) / dt)

    if not lin_accels:
        return 0.0, 0.0

    lin_rms = math.sqrt(sum(a * a for a in lin_accels) / len(lin_accels))
    ang_rms = math.sqrt(sum(a * a for a in ang_accels) / len(ang_accels))
    return lin_rms, ang_rms


def compute_path_efficiency(odoms, plan_poses):
    """Ratio of actual distance traveled to planned path length."""
    if len(odoms) < 2 or len(plan_poses) < 2:
        return 0.0

    actual = 0.0
    for i in range(1, len(odoms)):
        dx = odoms[i][1] - odoms[i - 1][1]
        dy = odoms[i][2] - odoms[i - 1][2]
        actual += math.hypot(dx, dy)

    planned = 0.0
    for i in range(1, len(plan_poses)):
        dx = plan_poses[i][0] - plan_poses[i - 1][0]
        dy = plan_poses[i][1] - plan_poses[i - 1][1]
        planned += math.hypot(dx, dy)

    if planned < 1e-6:
        return 0.0
    return actual / planned


def main():
    parser = argparse.ArgumentParser(description="Compute navigation KPIs from a hardware rosbag")
    parser.add_argument("bag_path", help="Path to the rosbag directory")
    parser.add_argument("--controller", default="unknown", help="Controller name for CSV")
    parser.add_argument("--planner", default="navfn", help="Planner name for CSV")
    parser.add_argument("--run", type=int, default=1, help="Run number for CSV")
    parser.add_argument("--csv", default="kpi_results.csv", help="CSV output file (append mode)")
    parser.add_argument("--odom-topic", default="/odom_opti",
                        help="Odometry topic (default: /odom_opti, OptiTrack)")
    parser.add_argument("--plan-topic", default="/plan",
                        help="Global plan topic (default: /plan, Dashing navfn_planner)")
    parser.add_argument("--map-odom-x", type=float, default=DEFAULT_MAP_ODOM_X,
                        help="map->odom X (must match localization_optitrack.launch.py)")
    parser.add_argument("--map-odom-y", type=float, default=DEFAULT_MAP_ODOM_Y,
                        help="map->odom Y (must match localization_optitrack.launch.py)")
    parser.add_argument("--map-odom-yaw", type=float, default=DEFAULT_MAP_ODOM_YAW,
                        help="map->odom yaw in RADIANS (must match localization_optitrack.launch.py)")
    args = parser.parse_args()

    if not os.path.exists(args.bag_path):
        print(f"Error: bag path '{args.bag_path}' not found")
        sys.exit(1)

    cmd_vel_topic = "/cmd_vel"
    print(f"Reading bag: {args.bag_path}")
    print(f"  Plan topic:    {args.plan_topic}")
    print(f"  Odom topic:    {args.odom_topic}")
    print(f"  Cmd_vel topic: {cmd_vel_topic}")
    plans, odoms, cmd_vels, odom_frame = read_bag(
        args.bag_path, args.plan_topic, args.odom_topic, cmd_vel_topic)

    print(f"  {args.plan_topic} messages:    {len(plans)}")
    print(f"  {args.odom_topic} messages:    {len(odoms)}")
    print(f"  {cmd_vel_topic} messages: {len(cmd_vels)}")

    if not plans:
        print(f"Error: no {args.plan_topic} messages found in bag")
        sys.exit(1)

    _, plan_frame, plan_poses = plans[-1]
    print(f"  Reference plan: {len(plan_poses)} poses (frame '{plan_frame}')")

    # Bring odometry into the plan's frame before measuring distances to it
    if plan_frame == "map" and odom_frame == "odom":
        print(f"  Transforming odom -> map with (x={args.map_odom_x}, "
              f"y={args.map_odom_y}, yaw={args.map_odom_yaw} rad)")
        odoms = odom_to_map(odoms, args.map_odom_x, args.map_odom_y, args.map_odom_yaw)
    elif odoms and plan_frame != odom_frame:
        print(f"  [WARN] plan frame '{plan_frame}' != odom frame '{odom_frame}' "
              f"and no transform is known -- CTE will be wrong")

    cte_rmse, cte_max = compute_cte(odoms, plan_poses)
    mission_time = compute_mission_time(cmd_vels)
    lin_accel_rms, ang_accel_rms = compute_accel_rms(cmd_vels)
    path_eff = compute_path_efficiency(odoms, plan_poses)

    print()
    print("=" * 55)
    print(f"  Controller:              {args.controller}")
    print(f"  Planner:                 {args.planner}")
    print(f"  Run:                     {args.run}")
    print("-" * 55)
    print(f"  CTE RMSE:                {cte_rmse:.4f} m")
    print(f"  CTE Max:                 {cte_max:.4f} m")
    print(f"  Mission Time:            {mission_time:.2f} s")
    print(f"  Linear Accel RMS:        {lin_accel_rms:.4f} m/s^2")
    print(f"  Angular Accel RMS:       {ang_accel_rms:.4f} rad/s^2")
    print(f"  Path Efficiency:         {path_eff:.4f}")
    print("=" * 55)

    write_header = not os.path.exists(args.csv)
    with open(args.csv, "a", newline="") as f:
        writer = csv.writer(f)
        if write_header:
            writer.writerow([
                "controller", "planner", "run", "cte_rmse", "cte_max",
                "mission_time", "linear_accel_rms", "angular_accel_rms",
                "path_efficiency",
            ])
        writer.writerow([
            args.controller, args.planner, args.run,
            f"{cte_rmse:.4f}", f"{cte_max:.4f}", f"{mission_time:.2f}",
            f"{lin_accel_rms:.4f}", f"{ang_accel_rms:.4f}", f"{path_eff:.4f}",
        ])

    print(f"\nResults appended to {args.csv}")


if __name__ == "__main__":
    main()
