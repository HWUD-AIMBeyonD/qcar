#!/usr/bin/env python3
"""Re-publish /tf_static periodically so a Dashing rosbag2 recording catches it.

Dashing's `ros2 bag record` subscribes with VOLATILE durability, so latched
(TRANSIENT_LOCAL) messages published before the recorder started are never
delivered to it. /tf_static is exactly that: robot_state_publisher and the
static map->odom publisher (localization_optitrack.launch.py) send it once at
startup, and a bag started later has no static transforms -- Foxglove then
can't place base->lidar, map->odom etc.

This node subscribes to /tf_static as TRANSIENT_LOCAL (so it gets the latched
transforms), keeps the latest transform per child frame, and re-publishes the
whole set on /tf_static every --period seconds. It publishes nothing new --
only the transforms already on /tf_static -- so it doesn't change the TF tree.

/map needs no relay: direct_map_publisher already republishes it every 2 s.

Usage (started by record_foxglove_bag.sh, or by hand):
    python3 tf_static_relay.py [--period 1.0]
"""

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from tf2_msgs.msg import TFMessage


class TfStaticRelay(Node):

    def __init__(self, period):
        super().__init__('tf_static_relay')

        latched_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=100
        )

        # child_frame_id -> TransformStamped
        self.transforms = {}

        self.publisher = self.create_publisher(TFMessage, '/tf_static', latched_qos)
        self.subscription = self.create_subscription(
            TFMessage, '/tf_static', self.tf_static_callback, latched_qos)
        self.timer = self.create_timer(period, self.republish)

        self.get_logger().info('Relaying /tf_static every %.1f s' % period)

    def tf_static_callback(self, msg):
        # Our own republished messages come back here too; storing by child
        # frame makes that a no-op.
        new_frames = []
        for t in msg.transforms:
            if t.child_frame_id not in self.transforms:
                new_frames.append('%s->%s' % (t.header.frame_id, t.child_frame_id))
            self.transforms[t.child_frame_id] = t
        if new_frames:
            self.get_logger().info('Static TF: ' + ', '.join(new_frames))

    def republish(self):
        if not self.transforms:
            return
        msg = TFMessage()
        msg.transforms = list(self.transforms.values())
        self.publisher.publish(msg)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--period', type=float, default=1.0,
                        help='republish period in seconds (default 1.0)')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = TfStaticRelay(args.period)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
