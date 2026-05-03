#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import math


class SimpleEKF(Node):
    def __init__(self):
        super().__init__('simple_ekf')

        # State: [x, y, theta, v]
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0

        # # IMU yaw rate
        # self.imu_yaw_rate = 0.0
        # self.last_imu_time = None

        # # Encoder velocity
        # self.encoder_velocity = 0.0
        # self.last_odom_time = None

        # Last update time
        self.last_update_time = self.get_clock().now()

        # Path tracking
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'odom'
        self.last_path_x = 0.0
        self.last_path_y = 0.0
        self.path_min_distance = 0.02  # 2cm minimum movement

        # QoS
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

        # --- ORIGINAL SUBSCRIBERS (commented out — using OptiTrack instead) ---
        # self.odom_sub = self.create_subscription(
        #     Odometry,
        #     '/odom_raw',
        #     self.odom_callback,
        #     qos
        # )
        #
        # self.imu_sub = self.create_subscription(
        #     Imu,
        #     '/imu',
        #     self.imu_callback,
        #     qos
        # )

        # --- OptiTrack odometry subscription (replaces encoder + IMU) ---
        self.opti_sub = self.create_subscription(
            Odometry,
            '/odom_opti',
            self.opti_callback,
            qos
        )

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odometry/filtered', qos)
        self.path_pub = self.create_publisher(Path, '/path_fused', 10)
        self.tf_pub_reliable = self.create_publisher(TFMessage, '/tf', tf_qos_reliable)
        self.tf_pub_best_effort = self.create_publisher(TFMessage, '/tf', tf_qos_best_effort)

        # --- ORIGINAL TIMER (commented out — publishing driven by /odom_opti callback) ---
        # self.timer = self.create_timer(0.02, self.update_and_publish)

        self.get_logger().info('Simple EKF started (OptiTrack mode)')
        self.get_logger().info('  Subscribing to: /odom_opti')
        self.get_logger().info('  Publishing to: /odometry/filtered, /path_fused, /tf')

    # --- ORIGINAL CALLBACKS (commented out) ---
    # def odom_callback(self, msg):
    #     """Get velocity from wheel encoders."""
    #     self.encoder_velocity = msg.twist.twist.linear.x
    #     self.last_odom_time = self.get_clock().now()
    #
    # def imu_callback(self, msg):
    #     """Get yaw rate from IMU gyroscope."""
    #     self.imu_yaw_rate = msg.angular_velocity.z
    #     self.last_imu_time = self.get_clock().now()

    # --- ORIGINAL UPDATE LOOP (commented out) ---
    # def update_and_publish(self):
    #     """Update state estimate and publish."""
    #     current_time = self.get_clock().now()
    #     dt = (current_time - self.last_update_time).nanoseconds / 1e9
    #
    #     # Debug print
    #     self.get_logger().info(f'dt={dt:.3f}, v={self.encoder_velocity:.3f}, yaw_rate={self.imu_yaw_rate:.3f}', throttle_duration_sec=1.0)
    #
    #     if dt <= 0 or dt > 0.5:
    #         self.last_update_time = current_time
    #         return
    #
    #     # Update theta using IMU yaw rate (much more accurate than encoder-derived)
    #     self.theta += self.imu_yaw_rate * dt
    #     self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))  # Normalize
    #
    #     # Update position using encoder velocity and IMU-corrected heading
    #     self.v = self.encoder_velocity
    #     self.x += self.v * math.cos(self.theta) * dt
    #     self.y += self.v * math.sin(self.theta) * dt
    #
    #     # Publish odometry
    #     self.publish_odometry(current_time)
    #
    #     # Publish TF
    #     self.publish_tf(current_time)
    #
    #     # Publish path
    #     self.publish_path(current_time)
    #
    #     self.last_update_time = current_time

    def opti_callback(self, msg: Odometry):
        """Receive OptiTrack odometry and republish as /odometry/filtered."""
        current_time = self.get_clock().now()

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.theta = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        self.v = msg.twist.twist.linear.x

        self.publish_odometry(current_time, msg)
        self.publish_tf(current_time, msg)
        self.publish_path(current_time)

    def publish_odometry(self, current_time, opti_msg: Odometry):
        """Republish OptiTrack odom as /odometry/filtered with correct frames."""
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base'

        odom_msg.pose = opti_msg.pose
        odom_msg.twist = opti_msg.twist

        self.odom_pub.publish(odom_msg)

    # --- ORIGINAL publish_odometry (commented out) ---
    # def publish_odometry(self, current_time):
    #     """Publish fused odometry."""
    #     odom_msg = Odometry()
    #     odom_msg.header.stamp = current_time.to_msg()
    #     odom_msg.header.frame_id = 'odom'
    #     odom_msg.child_frame_id = 'base'
    #
    #     # Position
    #     odom_msg.pose.pose.position.x = self.x
    #     odom_msg.pose.pose.position.y = self.y
    #     odom_msg.pose.pose.position.z = 0.0
    #
    #     # Orientation
    #     quat = self.euler_to_quaternion(0, 0, self.theta)
    #     odom_msg.pose.pose.orientation.x = quat[0]
    #     odom_msg.pose.pose.orientation.y = quat[1]
    #     odom_msg.pose.pose.orientation.z = quat[2]
    #     odom_msg.pose.pose.orientation.w = quat[3]
    #
    #     # Velocity
    #     odom_msg.twist.twist.linear.x = self.v
    #     odom_msg.twist.twist.angular.z = self.imu_yaw_rate
    #
    #     # Covariance
    #     odom_msg.pose.covariance = [0.0] * 36
    #     odom_msg.pose.covariance[0] = 0.1   # x
    #     odom_msg.pose.covariance[7] = 0.1   # y
    #     odom_msg.pose.covariance[35] = 0.05 # yaw
    #
    #     odom_msg.twist.covariance = [0.0] * 36
    #     odom_msg.twist.covariance[0] = 0.1   # vx
    #     odom_msg.twist.covariance[35] = 0.05 # vyaw
    #
    #     self.odom_pub.publish(odom_msg)

    def publish_tf(self, current_time, opti_msg: Odometry):
        """Publish odom -> base transform from OptiTrack data."""
        tf_msg = TFMessage()

        t1 = TransformStamped()
        t1.header.stamp = current_time.to_msg()
        t1.header.frame_id = 'odom'
        t1.child_frame_id = 'base'
        t1.transform.translation.x = opti_msg.pose.pose.position.x
        t1.transform.translation.y = opti_msg.pose.pose.position.y
        t1.transform.translation.z = opti_msg.pose.pose.position.z
        t1.transform.rotation = opti_msg.pose.pose.orientation
        tf_msg.transforms.append(t1)

        self.tf_pub_reliable.publish(tf_msg)
        self.tf_pub_best_effort.publish(tf_msg)

    # --- ORIGINAL publish_tf (commented out) ---
    # def publish_tf(self, current_time):
    #     """Publish odom -> base AND odom -> /base transforms."""
    #     tf_msg = TFMessage()
    #
    #     quat = self.euler_to_quaternion(0, 0, self.theta)
    #
    #     # Transform 1: odom -> base (for Cartographer)
    #     t1 = TransformStamped()
    #     t1.header.stamp = current_time.to_msg()
    #     t1.header.frame_id = 'odom'
    #     t1.child_frame_id = 'base'
    #     t1.transform.translation.x = self.x
    #     t1.transform.translation.y = self.y
    #     t1.transform.translation.z = 0.0
    #     t1.transform.rotation.x = quat[0]
    #     t1.transform.rotation.y = quat[1]
    #     t1.transform.rotation.z = quat[2]
    #     t1.transform.rotation.w = quat[3]
    #     tf_msg.transforms.append(t1)
    #
    #     # Transform 2: odom -> /base (for robot_state_publisher's frames)
    #     # t2 = TransformStamped()
    #     # t2.header.stamp = current_time.to_msg()
    #     # t2.header.frame_id = 'odom'
    #     # t2.child_frame_id = '/base'
    #     # t2.transform.translation.x = self.x
    #     # t2.transform.translation.y = self.y
    #     # t2.transform.translation.z = 0.0
    #     # t2.transform.rotation.x = quat[0]
    #     # t2.transform.rotation.y = quat[1]
    #     # t2.transform.rotation.z = quat[2]
    #     # t2.transform.rotation.w = quat[3]
    #     # tf_msg.transforms.append(t2)
    #
    #     self.tf_pub_reliable.publish(tf_msg)
    #     self.tf_pub_best_effort.publish(tf_msg)

    def publish_path(self, current_time):
        """Publish path for RViz visualization."""
        dx = self.x - self.last_path_x
        dy = self.y - self.last_path_y
        distance = math.sqrt(dx*dx + dy*dy)

        if distance >= self.path_min_distance:
            pose = PoseStamped()
            pose.header.stamp = current_time.to_msg()
            pose.header.frame_id = 'odom'
            pose.pose.position.x = self.x
            pose.pose.position.y = self.y
            pose.pose.position.z = 0.0

            quat = self.euler_to_quaternion(0, 0, self.theta)
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]

            self.path_msg.poses.append(pose)
            self.last_path_x = self.x
            self.last_path_y = self.y

        self.path_msg.header.stamp = current_time.to_msg()
        self.path_pub.publish(self.path_msg)

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy
        return [qx, qy, qz, qw]


def main(args=None):
    rclpy.init(args=args)
    node = SimpleEKF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
