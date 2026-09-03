import json
import math
import urllib.error
import urllib.request

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
# no tf2_ros python on the QCar's Dashing install -- publish TFMessage on /tf
# by hand, the same way qcar_odom/simple_ekf and qcar_amcl do
from tf2_msgs.msg import TFMessage

WARN_THROTTLE_SEC = 2.0


def quaternion_from_yaw(yaw):
    """Quaternion (x, y, z, w) for a rotation of `yaw` radians about Z."""
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


class HttpOdomNode(Node):
    def __init__(self):
        super().__init__('http_odom_node')

        self.declare_parameter('pose_url', 'http://192.168.0.3:8000/QCar/pose')
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('request_timeout', 0.1)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('child_frame', 'base')
        self.declare_parameter('publish_tf', True)

        self.pose_url = self.get_parameter('pose_url').value
        self.request_timeout = self.get_parameter('request_timeout').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        rate_hz = self.get_parameter('rate_hz').value

        self.prev_time = None
        self.prev_x = None
        self.prev_y = None
        self.prev_yaw = None
        self.last_warn_time = {}

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # TF QoS (dual for compatibility)
        tf_qos_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=100
        )
        tf_qos_best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=100
        )

        self.odom_pub = self.create_publisher(Odometry, '/odom_opti', qos)

        if self.publish_tf:
            self.tf_pub_reliable = self.create_publisher(
                TFMessage, '/tf', tf_qos_reliable)
            self.tf_pub_best_effort = self.create_publisher(
                TFMessage, '/tf', tf_qos_best_effort)
        else:
            self.tf_pub_reliable = None
            self.tf_pub_best_effort = None

        self.timer = self.create_timer(1.0 / rate_hz, self.poll_cb)

        self.get_logger().info(
            'Polling {} at {:.1f} Hz -> publishing /odom_opti'.format(
                self.pose_url, rate_hz))

    def warn_throttled(self, key, msg):
        """Dashing's logger has no throttle_duration_sec, so rate-limit here."""
        now = self.get_clock().now().nanoseconds * 1e-9
        last = self.last_warn_time.get(key)
        if last is None or now - last >= WARN_THROTTLE_SEC:
            self.last_warn_time[key] = now
            self.get_logger().warn(msg)

    def fetch_pose(self):
        """GET the pose endpoint, return (x, y, yaw_deg) or None on failure."""
        try:
            resp = urllib.request.urlopen(
                self.pose_url, timeout=self.request_timeout)
            try:
                payload = json.loads(resp.read().decode('utf-8'))
            finally:
                resp.close()
        except (urllib.error.URLError, OSError, ValueError) as exc:
            self.warn_throttled('fetch', 'pose fetch failed: {}'.format(exc))
            return None

        try:
            return (float(payload['x']),
                    float(payload['y']),
                    float(payload['yaw']))
        except (KeyError, TypeError, ValueError) as exc:
            self.warn_throttled(
                'payload',
                'unexpected pose payload {!r}: {}'.format(payload, exc))
            return None

    def poll_cb(self):
        sample = self.fetch_pose()
        if sample is None:
            return

        x, y, yaw_deg = sample
        yaw = math.radians(yaw_deg)

        # restamp with the local clock; the endpoint carries no usable stamp
        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9

        vx = 0.0
        vyaw = 0.0

        if self.prev_time is not None:
            dt = now_sec - self.prev_time
            if dt > 0.0:
                dx = x - self.prev_x
                dy = y - self.prev_y
                dist = math.hypot(dx, dy)
                # project displacement onto body heading for forward velocity
                heading = math.atan2(dy, dx)
                vx = dist / dt * math.cos(heading - yaw)
                dyaw = yaw - self.prev_yaw
                # wrap to [-pi, pi]
                dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))
                vyaw = dyaw / dt

        self.prev_time = now_sec
        self.prev_x = x
        self.prev_y = y
        self.prev_yaw = yaw

        qx, qy, qz, qw = quaternion_from_yaw(yaw)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.child_frame

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # mocap is very accurate
        odom.pose.covariance[0] = 0.001   # x
        odom.pose.covariance[7] = 0.001   # y
        odom.pose.covariance[14] = 0.001  # z
        odom.pose.covariance[21] = 0.001  # roll
        odom.pose.covariance[28] = 0.001  # pitch
        odom.pose.covariance[35] = 0.001  # yaw

        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vyaw

        odom.twist.covariance[0] = 0.01   # vx
        odom.twist.covariance[35] = 0.01  # vyaw

        self.odom_pub.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = odom.header.stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.child_frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            t.transform.rotation = odom.pose.pose.orientation

            tf_msg = TFMessage()
            tf_msg.transforms.append(t)
            self.tf_pub_reliable.publish(tf_msg)
            self.tf_pub_best_effort.publish(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HttpOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
