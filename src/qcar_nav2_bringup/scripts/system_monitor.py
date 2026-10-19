#!/usr/bin/env python3
"""Publish Jetson TX2 system load on /system_stats for the HW benchmarks.

Plain rclpy node, no psutil/numpy (Dashing, Python 3.6). Run with:
    python3 system_monitor.py

At 2 Hz publishes a std_msgs/String holding one JSON object:
    stamp           wall time (s)
    cpu             {"total": %, "per_core": {"cpu0": %, ...}}  (/proc/stat deltas)
    ram             {"used_mb", "total_mb", "swap_used_mb", "swap_total_mb"}
    gpu_load_raw    /sys/devices/gpu.0/load (0-1000 on TX2), null if missing
    gpu_load_pct    gpu_load_raw / 10
    thermal         [{"zone", "type", "temp_c"}, ...]
    cpu_freq_mhz    {"cpu0": MHz, ...}  (scaling_cur_freq; offline cores absent)
    procs           {label: {"pids", "cpu_pct", "rss_mb"}} -- cpu_pct is % of
                    ONE core summed over the label's processes, null on the
                    first sample or when nothing matched

CPU values are null on the first sample (no previous delta). Every file read
fails soft: a missing or unreadable file just yields null / an absent entry.
"""

import glob
import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import String

PUBLISH_PERIOD_SEC = 0.5

# label -> substrings searched for in /proc/<pid>/cmdline (space-joined)
PROCESS_PATTERNS = [
    ('hybrid_planner_server', ['hybrid_planner_server']),
    ('rpp_controller_server', ['rpp_controller_server']),
    ('stanley_controller_server', ['stanley_controller_server']),
    ('vector_pursuit_controller_server', ['vector_pursuit_controller_server']),
    ('nav2_costmap_2d', ['nav2_costmap_2d']),
    ('world_model', ['world_model']),
    ('bt_navigator', ['bt_navigator']),
    ('recoveries_node', ['recoveries_node']),
    ('http_odom_node', ['http_odom_node']),
    ('qcar_hardware_interface', ['qcar_hardware_interface']),
    ('rosbag2', ['ros2 bag record', 'rosbag2']),
]

# launchers whose cmdline merely mentions a node (sudo bash -c '... node ...',
# pgrep -f node); counting them would double the RSS of the real process
WRAPPER_EXES = ('sudo', 'bash', 'sh', 'pgrep', 'pkill', 'timeout', 'grep')

try:
    CLK_TCK = float(os.sysconf('SC_CLK_TCK'))
except (ValueError, OSError, AttributeError):
    CLK_TCK = 100.0
try:
    PAGE_SIZE = float(os.sysconf('SC_PAGE_SIZE'))
except (ValueError, OSError, AttributeError):
    PAGE_SIZE = 4096.0


def read_text(path):
    """Whole file as a stripped string, or None on any error."""
    try:
        with open(path, 'r') as f:
            return f.read().strip()
    except (IOError, OSError, UnicodeDecodeError):
        return None


def read_number(path, scale=1.0):
    text = read_text(path)
    if text is None:
        return None
    try:
        return float(text) * scale
    except ValueError:
        return None


def read_proc_stat():
    """{'cpu': (busy, total), 'cpu0': (busy, total), ...} in jiffies."""
    text = read_text('/proc/stat')
    result = {}
    if text is None:
        return result
    for line in text.splitlines():
        if not line.startswith('cpu'):
            continue
        parts = line.split()
        try:
            # user nice system idle iowait irq softirq steal (guest is in user)
            values = [int(v) for v in parts[1:9]]
        except ValueError:
            continue
        if len(values) < 4:
            continue
        total = sum(values)
        idle = values[3] + (values[4] if len(values) > 4 else 0)
        result[parts[0]] = (total - idle, total)
    return result


def cpu_percentages(prev, cur):
    """(total %, {core: %}) from two read_proc_stat() samples."""
    total = None
    per_core = {}
    for name, (busy, tot) in cur.items():
        if name not in prev:
            continue
        d_tot = tot - prev[name][1]
        if d_tot <= 0:
            continue
        pct = round(100.0 * (busy - prev[name][0]) / d_tot, 1)
        if name == 'cpu':
            total = pct
        else:
            per_core[name] = pct
    return total, per_core


def read_meminfo():
    text = read_text('/proc/meminfo')
    info = {}
    if text is None:
        return {'used_mb': None, 'total_mb': None,
                'swap_used_mb': None, 'swap_total_mb': None}
    for line in text.splitlines():
        key, _, rest = line.partition(':')
        try:
            info[key] = float(rest.split()[0]) / 1024.0  # kB -> MB
        except (IndexError, ValueError):
            pass

    total = info.get('MemTotal')
    available = info.get('MemAvailable')
    if available is None and 'MemFree' in info:
        available = (info['MemFree'] + info.get('Buffers', 0.0) +
                     info.get('Cached', 0.0))
    used = None
    if total is not None and available is not None:
        used = round(total - available, 1)

    swap_total = info.get('SwapTotal')
    swap_used = None
    if swap_total is not None and 'SwapFree' in info:
        swap_used = round(swap_total - info['SwapFree'], 1)

    return {
        'used_mb': used,
        'total_mb': round(total, 1) if total is not None else None,
        'swap_used_mb': swap_used,
        'swap_total_mb': round(swap_total, 1) if swap_total is not None else None,
    }


def read_thermal():
    zones = []
    for zone in sorted(glob.glob('/sys/devices/virtual/thermal/thermal_zone*')):
        temp = read_number(os.path.join(zone, 'temp'), 0.001)  # millidegrees
        if temp is None:
            continue
        zones.append({
            'zone': os.path.basename(zone),
            'type': read_text(os.path.join(zone, 'type')),
            'temp_c': round(temp, 1),
        })
    return zones


def read_cpu_freqs():
    freqs = {}
    for path in sorted(glob.glob('/sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq')):
        mhz = read_number(path, 0.001)  # kHz -> MHz
        if mhz is not None:
            freqs[path.split('/')[5]] = round(mhz, 1)
    return freqs


def read_cmdline(pid):
    try:
        with open('/proc/{}/cmdline'.format(pid), 'rb') as f:
            raw = f.read()
    except (IOError, OSError):
        return None
    args = [a.decode('utf-8', 'replace') for a in raw.split(b'\x00') if a]
    return args or None


def read_proc_ticks(pid):
    """utime + stime of a process in jiffies, or None."""
    text = read_text('/proc/{}/stat'.format(pid))
    if text is None:
        return None
    # comm (field 2) may contain spaces; everything after the last ')' is
    # fields 3.., so utime/stime (fields 14/15) are at index 11/12
    fields = text.rpartition(')')[2].split()
    try:
        return int(fields[11]) + int(fields[12])
    except (IndexError, ValueError):
        return None


def read_proc_rss_mb(pid):
    text = read_text('/proc/{}/statm'.format(pid))
    if text is None:
        return None
    try:
        return int(text.split()[1]) * PAGE_SIZE / (1024.0 * 1024.0)
    except (IndexError, ValueError):
        return None


def match_processes():
    """{label: [pid, ...]} for every running process we care about."""
    matches = dict((label, []) for label, _ in PROCESS_PATTERNS)
    own_pid = os.getpid()
    for entry in glob.glob('/proc/[0-9]*'):
        try:
            pid = int(os.path.basename(entry))
        except ValueError:
            continue
        if pid == own_pid:
            continue
        args = read_cmdline(pid)
        if not args or os.path.basename(args[0]) in WRAPPER_EXES:
            continue
        cmdline = ' '.join(args)
        for label, patterns in PROCESS_PATTERNS:
            if any(p in cmdline for p in patterns):
                matches[label].append(pid)
                break
    return matches


class SystemMonitor(Node):
    def __init__(self):
        super().__init__('system_monitor')
        self.pub = self.create_publisher(String, '/system_stats', QoSProfile(depth=10))
        self.prev_stat = read_proc_stat()
        self.prev_proc_ticks = {}
        self.prev_time = time.monotonic()
        self.timer = self.create_timer(PUBLISH_PERIOD_SEC, self.timer_cb)
        self.get_logger().info('Publishing /system_stats at {:.1f} Hz'.format(
            1.0 / PUBLISH_PERIOD_SEC))

    def process_stats(self, dt):
        procs = {}
        ticks_now = {}
        for label, pids in match_processes().items():
            cpu = 0.0
            have_cpu = False
            rss = 0.0
            have_rss = False
            for pid in pids:
                ticks = read_proc_ticks(pid)
                if ticks is not None:
                    ticks_now[pid] = ticks
                    prev = self.prev_proc_ticks.get(pid)
                    if prev is not None and dt > 0.0:
                        cpu += 100.0 * (ticks - prev) / CLK_TCK / dt
                        have_cpu = True
                rss_mb = read_proc_rss_mb(pid)
                if rss_mb is not None:
                    rss += rss_mb
                    have_rss = True
            procs[label] = {
                'pids': pids,
                'cpu_pct': round(cpu, 1) if have_cpu else None,
                'rss_mb': round(rss, 1) if have_rss else None,
            }
        self.prev_proc_ticks = ticks_now
        return procs

    def timer_cb(self):
        now = time.monotonic()
        dt = now - self.prev_time
        self.prev_time = now

        stat = read_proc_stat()
        cpu_total, per_core = cpu_percentages(self.prev_stat, stat)
        self.prev_stat = stat

        gpu_raw = read_number('/sys/devices/gpu.0/load')

        stats = {
            'stamp': time.time(),
            'cpu': {'total': cpu_total, 'per_core': per_core},
            'ram': read_meminfo(),
            'gpu_load_raw': gpu_raw,
            'gpu_load_pct': round(gpu_raw / 10.0, 1) if gpu_raw is not None else None,
            'thermal': read_thermal(),
            'cpu_freq_mhz': read_cpu_freqs(),
            'procs': self.process_stats(dt),
        }
        msg = String()
        msg.data = json.dumps(stats, sort_keys=True)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
