#!/usr/bin/env python3
"""
Custom AMCL node for QCar on ROS2 Dashing.
Uses numpy arrays for particles and Dashing-compatible TF publishing.
"""

import math
import numpy as np
from threading import Lock

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, TransformStamped
from tf2_msgs.msg import TFMessage

import tf2_ros


class QCarAMCL(Node):
    def __init__(self):
        super().__init__('qcar_amcl')

        # Parameters
        self.declare_parameter('base_frame_id', 'base')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('global_frame_id', 'map')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('num_particles', 500)
        self.declare_parameter('update_rate', 10.0)

        # Motion model parameters
        self.declare_parameter('alpha1', 0.2)  # Rotation noise from rotation
        self.declare_parameter('alpha2', 0.2)  # Rotation noise from translation
        self.declare_parameter('alpha3', 0.2)  # Translation noise from translation
        self.declare_parameter('alpha4', 0.2)  # Translation noise from rotation

        self.base_frame = self.get_parameter('base_frame_id').value
        self.odom_frame = self.get_parameter('odom_frame_id').value
        self.global_frame = self.get_parameter('global_frame_id').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.num_particles = self.get_parameter('num_particles').value
        self.update_rate = self.get_parameter('update_rate').value

        self.alpha1 = self.get_parameter('alpha1').value
        self.alpha2 = self.get_parameter('alpha2').value
        self.alpha3 = self.get_parameter('alpha3').value
        self.alpha4 = self.get_parameter('alpha4').value

        self.get_logger().info(f'Frames: {self.global_frame} -> {self.odom_frame} -> {self.base_frame}')
        self.get_logger().info(f'Particles: {self.num_particles}, Update rate: {self.update_rate} Hz')

        # Particle filter state
        self.particles = None  # Nx3 array [x, y, theta]
        self.weights = None
        self.initialized = False
        self.lock = Lock()

        # Map
        self.map_data = None
        self.map_info = None
        self.map_received = False

        # Odometry tracking
        self.last_odom_x = None
        self.last_odom_y = None
        self.last_odom_theta = None

        # TF Buffer and Listener for reading transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # TF publishers (Dashing compatible - dual QoS)
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
        self.tf_pub_reliable = self.create_publisher(TFMessage, '/tf', tf_qos_reliable)
        self.tf_pub_best_effort = self.create_publisher(TFMessage, '/tf', tf_qos_best_effort)

        # QoS Profiles
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        scan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        default_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos
        )

        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, scan_qos
        )

        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.initial_pose_callback, default_qos
        )

        # Publishers
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/amcl_pose', default_qos
        )

        self.particle_pub = self.create_publisher(
            PoseArray, '/particlecloud', default_qos
        )

        # Timer for updates
        self.timer = self.create_timer(1.0 / self.update_rate, self.timer_callback)

        self.get_logger().info('QCar AMCL started - waiting for map...')

    def map_callback(self, msg):
        """Handle incoming map."""
        with self.lock:
            self.map_info = msg.info
            width = msg.info.width
            height = msg.info.height
            self.map_data = np.array(msg.data, dtype=np.int8).reshape((height, width))
            self.map_received = True
        self.get_logger().info(f'Map received: {width}x{height}, resolution={msg.info.resolution}')

    def scan_callback(self, msg):
        """Handle incoming laser scan - used for weight updates."""
        if not self.initialized or not self.map_received:
            return

        # For now, just trigger a resample occasionally
        # Full implementation would do ray casting and weight updates
        pass

    def initial_pose_callback(self, msg):
        """Handle initial pose estimate from RViz."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        # Get covariance for initialization spread
        cov = msg.pose.covariance
        cov_x = max(0.1, math.sqrt(abs(cov[0])))    # xx
        cov_y = max(0.1, math.sqrt(abs(cov[7])))    # yy
        cov_theta = max(0.05, math.sqrt(abs(cov[35])))  # theta-theta

        self.get_logger().info(f'Initial pose: ({x:.2f}, {y:.2f}, {math.degrees(theta):.1f}deg)')
        self.get_logger().info(f'Covariance: x={cov_x:.2f}, y={cov_y:.2f}, theta={cov_theta:.2f}')

        # Initialize particles
        with self.lock:
            self.particles = np.zeros((self.num_particles, 3))
            self.particles[:, 0] = np.random.normal(x, cov_x, self.num_particles)
            self.particles[:, 1] = np.random.normal(y, cov_y, self.num_particles)
            self.particles[:, 2] = np.random.normal(theta, cov_theta, self.num_particles)
            self.weights = np.ones(self.num_particles) / self.num_particles
            self.initialized = True

        # Reset odometry tracking
        self.last_odom_x = None
        self.last_odom_y = None
        self.last_odom_theta = None

        self.get_logger().info(f'Particles initialized around ({x:.2f}, {y:.2f})')

    def motion_update(self, dx, dy, dtheta):
        """Update particles based on odometry motion."""
        if not self.initialized:
            return

        # Convert to robot-relative motion
        delta_trans = math.sqrt(dx * dx + dy * dy)

        # Skip tiny motions to avoid numerical issues
        if delta_trans < 0.001 and abs(dtheta) < 0.001:
            return

        delta_rot1 = math.atan2(dy, dx) if delta_trans > 0.01 else 0.0
        delta_rot2 = dtheta - delta_rot1

        with self.lock:
            for i in range(self.num_particles):
                # Add noise - use max() to ensure scale is always positive
                noisy_rot1 = delta_rot1 + np.random.normal(0, max(0.01,
                    self.alpha1 * abs(delta_rot1) + self.alpha2 * delta_trans))
                noisy_trans = delta_trans + np.random.normal(0, max(0.01,
                    self.alpha3 * delta_trans + self.alpha4 * (abs(delta_rot1) + abs(delta_rot2))))
                noisy_rot2 = delta_rot2 + np.random.normal(0, max(0.01,
                    self.alpha1 * abs(delta_rot2) + self.alpha2 * delta_trans))

                # Apply motion
                self.particles[i, 0] += noisy_trans * math.cos(self.particles[i, 2] + noisy_rot1)
                self.particles[i, 1] += noisy_trans * math.sin(self.particles[i, 2] + noisy_rot1)
                self.particles[i, 2] += noisy_rot1 + noisy_rot2

                # Normalize angle
                self.particles[i, 2] = math.atan2(
                    math.sin(self.particles[i, 2]),
                    math.cos(self.particles[i, 2])
                )

    def get_estimate(self):
        """Get current pose estimate from particles."""
        if not self.initialized:
            return None, None

        with self.lock:
            # Weighted mean
            x = np.average(self.particles[:, 0], weights=self.weights)
            y = np.average(self.particles[:, 1], weights=self.weights)

            # Circular mean for angle
            sin_sum = np.average(np.sin(self.particles[:, 2]), weights=self.weights)
            cos_sum = np.average(np.cos(self.particles[:, 2]), weights=self.weights)
            theta = math.atan2(sin_sum, cos_sum)

            # Covariance
            cov = np.cov(self.particles.T, aweights=self.weights)

        return (x, y, theta), cov

    def timer_callback(self):
        """Main update loop."""
        if not self.map_received:
            self.get_logger().warn('Waiting for map...', throttle_duration_sec=5.0)
            return

        if not self.initialized:
            self.get_logger().warn('Waiting for initial pose...', throttle_duration_sec=5.0)
            return

        # Get current odometry
        try:
            trans = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            odom_x = trans.transform.translation.x
            odom_y = trans.transform.translation.y
            q = trans.transform.rotation
            odom_theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                    1.0 - 2.0 * (q.y * q.y + q.z * q.z))

            # Motion update
            if self.last_odom_x is not None:
                dx = odom_x - self.last_odom_x
                dy = odom_y - self.last_odom_y
                dtheta = odom_theta - self.last_odom_theta

                # Normalize dtheta
                dtheta = math.atan2(math.sin(dtheta), math.cos(dtheta))

                self.motion_update(dx, dy, dtheta)

            self.last_odom_x = odom_x
            self.last_odom_y = odom_y
            self.last_odom_theta = odom_theta

        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=5.0)
            return

        # Get pose estimate
        estimate, covariance = self.get_estimate()
        if estimate is None:
            return

        x, y, theta = estimate

        # Publish pose
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.global_frame
        pose_msg.pose.pose.position.x = float(x)
        pose_msg.pose.pose.position.y = float(y)
        pose_msg.pose.pose.position.z = 0.0
        pose_msg.pose.pose.orientation.z = float(math.sin(theta / 2.0))
        pose_msg.pose.pose.orientation.w = float(math.cos(theta / 2.0))

        # Set covariance
        if covariance is not None and covariance.shape == (3, 3):
            pose_msg.pose.covariance[0] = float(covariance[0, 0])   # xx
            pose_msg.pose.covariance[7] = float(covariance[1, 1])   # yy
            pose_msg.pose.covariance[35] = float(covariance[2, 2])  # theta-theta

        self.pose_pub.publish(pose_msg)

        # Publish particles
        with self.lock:
            particle_msg = PoseArray()
            particle_msg.header.stamp = self.get_clock().now().to_msg()
            particle_msg.header.frame_id = self.global_frame

            for p in self.particles:
                pose = Pose()
                pose.position.x = float(p[0])
                pose.position.y = float(p[1])
                pose.orientation.z = float(math.sin(p[2] / 2.0))
                pose.orientation.w = float(math.cos(p[2] / 2.0))
                particle_msg.poses.append(pose)

        self.particle_pub.publish(particle_msg)

        # Publish TF: map -> odom (Dashing compatible)
        self.publish_tf(x, y, theta, odom_x, odom_y, odom_theta)

    def publish_tf(self, x, y, theta, odom_x, odom_y, odom_theta):
        """Publish map->odom transform (Dashing compatible using TFMessage)."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.global_frame
        t.child_frame_id = self.odom_frame

        # Calculate map->odom transform
        # map->base = estimate (x, y, theta)
        # odom->base = current odom (odom_x, odom_y, odom_theta)
        # map->odom = map->base * inverse(odom->base)

        diff_theta = theta - odom_theta
        cos_diff = math.cos(diff_theta)
        sin_diff = math.sin(diff_theta)

        # Transform odom origin to map frame
        t.transform.translation.x = float(x - (odom_x * cos_diff - odom_y * sin_diff))
        t.transform.translation.y = float(y - (odom_x * sin_diff + odom_y * cos_diff))
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = float(math.sin(diff_theta / 2.0))
        t.transform.rotation.w = float(math.cos(diff_theta / 2.0))

        # Publish via TFMessage (Dashing compatible)
        msg = TFMessage()
        msg.transforms = [t]
        self.tf_pub_reliable.publish(msg)
        self.tf_pub_best_effort.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = QCarAMCL()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

