#!/usr/bin/env python3
"""Planner-logic vs costmap check for the hybrid planner (Dashing, Python 3.6).

Grabs one message from every nav_msgs/OccupancyGrid topic (or the ones given
with --topics) and, for each grid, reports its frame/size/origin, the cell
values at the start and goal, and whether start and goal are connected
through cells the hybrid planner would treat as free.

The free/blocked rule mirrors HybridPlanner::isCollision() as seen through
Costmap2DPublisher's 0..100 translation:
    -1  (NO_INFORMATION) -> blocked
    100 (LETHAL)         -> blocked
    99  (INSCRIBED)      -> blocked, except within --escape m of the start
    off the grid         -> blocked
    anything else        -> free

If the planner's costmap says "connected" but the planner still fails, the
search logic is at fault. If it says "not connected" / "off grid", the
costmap is.

Usage (with the nav stack running):
    python3 check_costmap.py
    python3 check_costmap.py --goal 0.5699 2.6435 --start 0.0 0.0
    python3 check_costmap.py --topics /map /global_costmap/costmap
"""

import argparse
import collections
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid

GRID_TYPE = 'nav_msgs/msg/OccupancyGrid'


class Grabber(Node):
    def __init__(self, topics):
        super().__init__('costmap_checker')
        # Volatile matches both volatile (costmaps) and transient-local (/map)
        # publishers; direct_map_publisher re-sends /map every 2 s anyway.
        qos = QoSProfile(depth=1,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.VOLATILE)
        self.grids = {}
        for t in topics:
            self.create_subscription(OccupancyGrid, t, self._cb(t), qos)

    def _cb(self, topic):
        def cb(msg):
            self.grids[topic] = msg
        return cb


def find_grid_topics(node):
    out = []
    for name, types in node.get_topic_names_and_types():
        if any(t.replace('/msg/', '/') == GRID_TYPE.replace('/msg/', '/') for t in types):
            out.append(name)
    return sorted(out)


def lookup_start(node, timeout):
    """Robot pose in map from TF (map -> base), or None."""
    try:
        from tf2_ros.buffer import Buffer
        from tf2_ros.transform_listener import TransformListener
    except ImportError:
        return None
    buf = Buffer()
    TransformListener(buf, node)
    end = time.time() + timeout
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            tf = buf.lookup_transform('map', 'base', rclpy.time.Time())
            return tf.transform.translation.x, tf.transform.translation.y
        except Exception:  # noqa: BLE001 -- not available yet
            continue
    return None


def to_cell(msg, x, y):
    info = msg.info
    mx = int(math.floor((x - info.origin.position.x) / info.resolution))
    my = int(math.floor((y - info.origin.position.y) / info.resolution))
    if 0 <= mx < info.width and 0 <= my < info.height:
        return mx, my
    return None


def analyse(topic, msg, start, goal, escape):
    info = msg.info
    w, h, res = info.width, info.height, info.resolution
    ox, oy = info.origin.position.x, info.origin.position.y
    data = msg.data

    counts = collections.Counter()
    for v in data:
        if v == -1:
            counts['unknown(-1)'] += 1
        elif v == 0:
            counts['free(0)'] += 1
        elif v == 99:
            counts['inscribed(99)'] += 1
        elif v == 100:
            counts['lethal(100)'] += 1
        else:
            counts['inflated(1-98)'] += 1

    print('=' * 70)
    print('{}  frame={}  {}x{} cells @ {:.3f} m'.format(
        topic, msg.header.frame_id, w, h, res))
    print('  origin=({:.2f}, {:.2f})  covers x[{:.2f}, {:.2f}] y[{:.2f}, {:.2f}]'.format(
        ox, oy, ox, ox + w * res, oy, oy + h * res))
    print('  cells: ' + ', '.join('{}={}'.format(k, counts[k]) for k in sorted(counts)))

    def blocked(cx, cy):
        v = data[cy * w + cx]
        if v == -1 or v == 100:
            return True
        if v >= 99:
            wx = ox + (cx + 0.5) * res
            wy = oy + (cy + 0.5) * res
            return math.hypot(wx - start[0], wy - start[1]) > escape
        return False

    sc = to_cell(msg, *start)
    gc = to_cell(msg, *goal)
    for label, pt, c in (('start', start, sc), ('goal', goal, gc)):
        if c is None:
            print('  {} ({:.2f}, {:.2f}): OFF GRID'.format(label, pt[0], pt[1]))
        else:
            print('  {} ({:.2f}, {:.2f}): cell {} value={} {}'.format(
                label, pt[0], pt[1], c, data[c[1] * w + c[0]],
                'BLOCKED' if blocked(*c) else 'free'))

    if sc is None or gc is None:
        print('  VERDICT: start/goal off this grid -> planner cannot plan on it')
        return
    if blocked(*gc):
        print('  VERDICT: goal cell blocked -> "Goal ... is in collision or off the map"')
        return

    # 8-connected flood from the goal, like computeHeuristic()
    seen = bytearray(w * h)
    q = collections.deque([gc])
    seen[gc[1] * w + gc[0]] = 1
    while q:
        cx, cy = q.popleft()
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < w and 0 <= ny < h and not seen[ny * w + nx] and not blocked(nx, ny):
                seen[ny * w + nx] = 1
                q.append((nx, ny))
    reached = sum(seen)
    if seen[sc[1] * w + sc[0]]:
        print('  VERDICT: start and goal CONNECTED ({} cells reachable from goal)'.format(reached))
        print('           -> if the planner still fails on this costmap, it is the planner logic')
    else:
        print('  VERDICT: start NOT reachable from goal ({} cells reachable from goal)'.format(reached))
        print('           -> costmap problem: "Goal is not reachable from the start"')


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--topics', nargs='*', help='grid topics (default: all OccupancyGrid topics)')
    p.add_argument('--goal', nargs=2, type=float, default=[0.5699, 2.6435])
    p.add_argument('--start', nargs=2, type=float, help='default: TF map->base')
    p.add_argument('--escape', type=float, default=0.35, help='start_escape_radius')
    p.add_argument('--wait', type=float, default=6.0, help='seconds to collect grids')
    args = p.parse_args()

    rclpy.init()
    probe = Node('costmap_checker_probe')
    time.sleep(1.0)  # let discovery fill in
    topics = args.topics or find_grid_topics(probe)
    probe.destroy_node()
    if not topics:
        print('No OccupancyGrid topics found.')
        rclpy.shutdown()
        return
    print('Grid topics: ' + ', '.join(topics))

    node = Grabber(topics)
    start = tuple(args.start) if args.start else lookup_start(node, 3.0)
    if start is None:
        print('Could not get map->base from TF; pass --start X Y')
        rclpy.shutdown()
        return
    print('Start (map): ({:.3f}, {:.3f})   Goal: ({:.3f}, {:.3f})'.format(
        start[0], start[1], args.goal[0], args.goal[1]))

    end = time.time() + args.wait
    while time.time() < end and len(node.grids) < len(topics):
        rclpy.spin_once(node, timeout_sec=0.1)

    for t in topics:
        if t in node.grids:
            analyse(t, node.grids[t], start, tuple(args.goal), args.escape)
        else:
            print('=' * 70)
            print('{}: NO MESSAGE in {:.0f} s (nothing publishing, or QoS mismatch)'.format(
                t, args.wait))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
