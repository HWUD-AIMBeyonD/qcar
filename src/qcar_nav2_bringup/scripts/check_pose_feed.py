#!/usr/bin/env python3
"""Pre-test check of the OptiTrack pose feed. Run on the car; reads only, changes nothing.

    python3 check_pose_feed.py http [--url URL] [--secs 30]   poll the endpoint like http_odom_node
    python3 check_pose_feed.py ros  [--secs 30]               measure /odom_opti (nav stack up)

http: run with http_odom_node stopped, so this is the only poller.
ros:  source ROS first; this is the number the controller actually lives with.

Limits come from the 12 clean hardware runs of Oct 19 (49-50 Hz, 8-9 ms requests,
100% OK, longest gap 121 ms). The RPP re-runs that went wrong had 17 Hz, 47 ms,
~90% OK and gaps up to 541 ms. Python 3.6 compatible; `http` mode needs only stdlib.
"""
import argparse
import json
import math
import time
import urllib.error
import urllib.request

URL = 'http://192.168.0.3:8000/QCar/pose'
RATE_HZ = 50.0
TIMEOUT_S = 0.1          # same as http_odom_node
MIN_RATE_HZ = 45.0
MAX_GAP_MS = 150.0
MAX_FETCH_P50_MS = 15.0
MAX_FETCH_P95_MS = 35.0
MAX_FAIL_PCT = 0.5
JUMP_M = 0.15            # one step this long with no long gap is a bad pose, not motion


def pct(values, p):
    s = sorted(values)
    return s[min(len(s) - 1, int(round(p / 100.0 * (len(s) - 1))))] if s else float('nan')


def poll_http(url, secs):
    """Return (samples [(t, x, y)], fetch times in ms, number of failed polls)."""
    samples, fetch_ms, failed = [], [], 0
    next_t = time.monotonic()
    end = next_t + secs
    while time.monotonic() < end:
        t0 = time.monotonic()
        try:
            resp = urllib.request.urlopen(url, timeout=TIMEOUT_S)
            try:
                p = json.loads(resp.read().decode('utf-8'))
            finally:
                resp.close()
            samples.append((time.monotonic(), float(p['x']), float(p['y'])))
        except (urllib.error.URLError, OSError, ValueError, KeyError, TypeError) as exc:
            failed += 1
            if failed <= 3:
                print('  failed poll: {}'.format(exc))
        fetch_ms.append((time.monotonic() - t0) * 1000.0)
        next_t = max(next_t + 1.0 / RATE_HZ, time.monotonic())
        time.sleep(max(0.0, next_t - time.monotonic()))
    return samples, fetch_ms, failed


def listen_ros(secs):
    """Return samples [(t, x, y)] received on /odom_opti."""
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
    rclpy.init()
    node = rclpy.create_node('pose_feed_check')
    samples = []

    def on_odom(msg):
        p = msg.pose.pose.position
        samples.append((time.monotonic(), p.x, p.y))

    sub = node.create_subscription(Odometry, '/odom_opti', on_odom, QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST, depth=100))
    end = time.monotonic() + secs
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_subscription(sub)
    node.destroy_node()
    rclpy.shutdown()
    return samples


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('mode', choices=['http', 'ros'])
    ap.add_argument('--url', default=URL)
    ap.add_argument('--secs', type=float, default=30.0)
    args = ap.parse_args()

    fetch_ms, failed = [], 0
    if args.mode == 'http':
        print('Polling {} at {:.0f} Hz for {:.0f} s...'.format(args.url, RATE_HZ, args.secs))
        samples, fetch_ms, failed = poll_http(args.url, args.secs)
    else:
        print('Listening to /odom_opti for {:.0f} s...'.format(args.secs))
        samples = listen_ros(args.secs)

    checks = []  # (name, value text, limit text, passed)
    if len(samples) < 2:
        print('\nFAIL: {} pose(s) received. Check the network, the server and the rigid body.'.format(len(samples)))
        return 1
    span = samples[-1][0] - samples[0][0]
    gaps = [(b[0] - a[0]) * 1000.0 for a, b in zip(samples, samples[1:])]
    steps = [math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(samples, samples[1:])]
    rate = (len(samples) - 1) / span
    jumps = sum(1 for s, g in zip(steps, gaps) if s > JUMP_M and g < MAX_GAP_MS)
    changed = sum(1 for s in steps if s > 0.0)

    checks.append(('Pose rate', '{:.1f} Hz'.format(rate), '>= {:.0f}'.format(MIN_RATE_HZ), rate >= MIN_RATE_HZ))
    checks.append(('Longest gap', '{:.0f} ms'.format(max(gaps)), '<= {:.0f}'.format(MAX_GAP_MS), max(gaps) <= MAX_GAP_MS))
    checks.append(('Position jumps', str(jumps), '0', jumps == 0))
    if fetch_ms:
        p50, p95 = pct(fetch_ms, 50), pct(fetch_ms, 95)
        fail_pct = 100.0 * failed / len(fetch_ms)
        checks.append(('Request time, typical', '{:.1f} ms'.format(p50), '<= {:.0f}'.format(MAX_FETCH_P50_MS), p50 <= MAX_FETCH_P50_MS))
        checks.append(('Request time, p95', '{:.1f} ms'.format(p95), '<= {:.0f}'.format(MAX_FETCH_P95_MS), p95 <= MAX_FETCH_P95_MS))
        checks.append(('Failed requests', '{:.1f}%'.format(fail_pct), '<= {}'.format(MAX_FAIL_PCT), fail_pct <= MAX_FAIL_PCT))

    print('\n{:<24}{:>12}{:>10}'.format('Check', 'Measured', 'Limit'))
    for name, value, limit, ok in checks:
        print('{:<24}{:>12}{:>10}  {}'.format(name, value, limit, 'ok' if ok else 'FAIL'))

    print('\nLast pose: x={:.3f} y={:.3f}'.format(samples[-1][1], samples[-1][2]))
    print('Poses that changed: {:.0f}%  (near 0% while the car is being moved = frozen feed)'.format(
        100.0 * changed / len(steps)))

    good = all(c[3] for c in checks)
    print('\n{}'.format('PASS: feed matches the clean runs.' if good else 'FAIL: do not record yet.'))
    return 0 if good else 1


if __name__ == '__main__':
    raise SystemExit(main())
