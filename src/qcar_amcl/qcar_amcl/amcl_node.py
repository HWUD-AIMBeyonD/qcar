#!/usr/bin/env python3
"""
Custom AMCL node for QCar on ROS2 Dashing (Jetson TX2).
Uses likelihood field model for sensor updates, low variance resampling
with random particle injection, and motion-thresholded updates.

Odometry comes from simple_ekf via TF (odom -> base).
No tf2_ros dependency — uses manual TFMessage subscription.
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

try:
    from scipy import ndimage
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False


class QCarAMCL(Node):
    def __init__(self):
        super().__init__('qcar_amcl')

        # =============================================
        # Parameters
        # =============================================
        self.declare_parameter('base_frame_id', 'base')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('global_frame_id', 'map')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('num_particles', 300)
        self.declare_parameter('update_rate', 10.0)

        # Motion model parameters (odometry noise)
        self.declare_parameter('alpha1', 0.4)   # Rotation noise from rotation
        self.declare_parameter('alpha2', 0.4)   # Rotation noise from translation
        self.declare_parameter('alpha3', 0.4)   # Translation noise from translation
        self.declare_parameter('alpha4', 0.4)   # Translation noise from rotation

        # Sensor model parameters
        self.declare_parameter('sigma_hit', 0.4)        # Sensor noise std dev (meters)
        self.declare_parameter('scan_subsample', 10)     # Use every Nth ray
        self.declare_parameter('z_hit', 0.95)            # Weight for hit model
        self.declare_parameter('z_rand', 0.05)           # Weight for random model

        # Update thresholds (only do sensor update after this much motion)
        self.declare_parameter('update_min_d', 0.02)      # meters
        self.declare_parameter('update_min_a', 0.05)     # radians (~8 deg)

        # Resampling
        self.declare_parameter('random_particle_pct', 0.05)  # 5% random injection

        # Lidar mounting angle offset (match URDF rpy yaw)
        self.declare_parameter('lidar_yaw_offset', 1.54)

        # Get parameter values
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

        self.sigma_hit = self.get_parameter('sigma_hit').value
        self.scan_subsample = self.get_parameter('scan_subsample').value
        self.z_hit = self.get_parameter('z_hit').value
        self.z_rand = self.get_parameter('z_rand').value

        self.update_min_d = self.get_parameter('update_min_d').value
        self.update_min_a = self.get_parameter('update_min_a').value

        self.random_particle_pct = self.get_parameter('random_particle_pct').value
        self.lidar_yaw_offset = self.get_parameter('lidar_yaw_offset').value

        self.get_logger().info(f'Frames: {self.global_frame} -> {self.odom_frame} -> {self.base_frame}')
        self.get_logger().info(f'Particles: {self.num_particles}, Update rate: {self.update_rate} Hz')
        self.get_logger().info(f'Sensor model: sigma={self.sigma_hit}, subsample={self.scan_subsample}')
        self.get_logger().info(f'Motion thresholds: d={self.update_min_d}m, a={self.update_min_a}rad')

        # =============================================
        # State
        # =============================================
        self.particles = None       # Nx3 array [x, y, theta]
        self.weights = None         # N array
        self.initialized = False
        self.lock = Lock()

        # Map
        self.map_data = None        # Occupancy grid as numpy array
        self.map_info = None        # Map metadata
        self.distance_map = None    # Precomputed distance transform
        self.map_received = False

        # Latest scan (stored, used when motion threshold met)
        self.latest_scan = None

        # Odometry from TF (odom->base, published by simple_ekf)
        self.latest_odom_x = 0.0
        self.latest_odom_y = 0.0
        self.latest_odom_theta = 0.0
        self.odom_tf_received = False

        # Odometry tracking for motion delta
        self.last_odom_x = None
        self.last_odom_y = None
        self.last_odom_theta = None

        # Sensor update tracking (only update after enough motion)
        self.last_update_x = None
        self.last_update_y = None
        self.last_update_theta = None

        # =============================================
        # TF publishers (Dashing compatible - dual QoS)
        # =============================================
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

        # TF subscriber (manual — no tf2_ros in Dashing)
        self.tf_sub = self.create_subscription(
            TFMessage, '/tf', self.tf_callback,
            QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=100
            )
        )

        # =============================================
        # QoS Profiles
        # =============================================
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

        # =============================================
        # Subscribers
        # =============================================
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos
        )

        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, scan_qos
        )

        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.initial_pose_callback, default_qos
        )

        # =============================================
        # Publishers
        # =============================================
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/amcl_pose', default_qos
        )

        self.particle_pub = self.create_publisher(
            PoseArray, '/particlecloud', default_qos
        )

        # =============================================
        # Timer
        # =============================================
        self.timer = self.create_timer(1.0 / self.update_rate, self.timer_callback)

        if not SCIPY_AVAILABLE:
            self.get_logger().warn('scipy not available — using ray casting (slower). '
                                   'Install with: sudo apt install python3-scipy')

        self.get_logger().info('QCar AMCL started — waiting for map...')

    # =================================================================
    # TF LISTENER (manual — replaces tf2_ros)
    # =================================================================

    def tf_callback(self, msg):
        """Listen for odom->base transform from simple_ekf."""
        for t in msg.transforms:
            if t.header.frame_id == self.odom_frame and t.child_frame_id == self.base_frame:
                self.latest_odom_x = t.transform.translation.x
                self.latest_odom_y = t.transform.translation.y
                q = t.transform.rotation
                self.latest_odom_theta = math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                )
                self.odom_tf_received = True

    # =================================================================
    # MAP
    # =================================================================

    def map_callback(self, msg):
        """Handle incoming map and precompute distance transform."""
        if self.map_received:
            return  # Already have the map, ignore republishes

        with self.lock:
            self.map_info = msg.info
            width = msg.info.width
            height = msg.info.height
            self.map_data = np.array(msg.data, dtype=np.int8).reshape((height, width))

            # Precompute distance transform for likelihood field model
            if SCIPY_AVAILABLE:
                occupied = (self.map_data > 50).astype(np.float32)
                self.distance_map = ndimage.distance_transform_edt(1 - occupied) * msg.info.resolution
                self.get_logger().info('Distance transform computed (likelihood field model)')
            else:
                self.distance_map = None
                self.get_logger().info('Using ray casting model (no scipy)')

            self.map_received = True

        self.get_logger().info(f'Map received: {width}x{height}, resolution={msg.info.resolution}m/px')

    # =================================================================
    # SCAN
    # =================================================================

    def scan_callback(self, msg):
        """Store latest scan. Sensor update happens in timer when motion threshold is met."""
        self.latest_scan = msg

    # =================================================================
    # INITIAL POSE
    # =================================================================

    def initial_pose_callback(self, msg):
        """Handle initial pose estimate from RViz."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        # Get covariance for initialization spread
        cov = msg.pose.covariance
        cov_x = max(0.1, math.sqrt(abs(cov[0])))
        cov_y = max(0.1, math.sqrt(abs(cov[7])))
        cov_theta = max(0.05, math.sqrt(abs(cov[35])))

        self.get_logger().info(f'Initial pose: ({x:.2f}, {y:.2f}, {math.degrees(theta):.1f}deg)')
        self.get_logger().info(f'Spread: x={cov_x:.2f}, y={cov_y:.2f}, theta={cov_theta:.2f}')

        with self.lock:
            self.particles = np.zeros((self.num_particles, 3))
            self.particles[:, 0] = np.random.normal(x, cov_x, self.num_particles)
            self.particles[:, 1] = np.random.normal(y, cov_y, self.num_particles)
            self.particles[:, 2] = np.random.normal(theta, cov_theta, self.num_particles)
            self.weights = np.ones(self.num_particles) / self.num_particles
            self.initialized = True

        # Reset odometry and update tracking
        self.last_odom_x = None
        self.last_odom_y = None
        self.last_odom_theta = None
        self.last_update_x = None
        self.last_update_y = None
        self.last_update_theta = None

        self.get_logger().info(f'Initialized {self.num_particles} particles around ({x:.2f}, {y:.2f})')

    # =================================================================
    # MOTION UPDATE
    # =================================================================

    def motion_update(self, dx, dy, dtheta):
        """Update particles based on odometry motion (sample motion model)."""
        if not self.initialized:
            return

        delta_trans = math.sqrt(dx * dx + dy * dy)

        # Skip tiny motions
        if delta_trans < 0.001 and abs(dtheta) < 0.001:
            return

        delta_rot1 = math.atan2(dy, dx) if delta_trans > 0.01 else 0.0
        delta_rot2 = dtheta - delta_rot1

        with self.lock:
            for i in range(self.num_particles):
                # Add noise proportional to motion
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

    # =================================================================
    # SENSOR UPDATE
    # =================================================================

    def sensor_update(self, scan_msg):
        """Update particle weights using laser scan against the map."""
        if not self.initialized or not self.map_received:
            return

        if self.distance_map is not None:
            self._sensor_update_likelihood_field(scan_msg)
        else:
            self._sensor_update_ray_cast(scan_msg)

    def _sensor_update_likelihood_field(self, scan_msg):
        """
        Likelihood field model — no ray casting needed.
        For each particle, project scan endpoints into map frame,
        look up precomputed nearest-obstacle distance, score.
        """
        ranges = np.array(scan_msg.ranges)
        angle_min = scan_msg.angle_min
        angle_inc = scan_msg.angle_increment

        res = self.map_info.resolution
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        h, w = self.distance_map.shape

        sigma = self.sigma_hit
        sigma_sq_2 = 2.0 * sigma * sigma
        z_hit = self.z_hit
        z_rand = self.z_rand
        max_range = scan_msg.range_max

        # Subsample scan rays for performance
        step = self.scan_subsample
        indices = []
        for idx in range(0, len(ranges), step):
            r = ranges[idx]
            if np.isfinite(r) and scan_msg.range_min < r < max_range:
                indices.append(idx)

        if len(indices) == 0:
            return

        # Precompute ray angles and ranges
        ray_angles = np.array([angle_min + idx * angle_inc for idx in indices])
        ray_ranges = np.array([ranges[idx] for idx in indices])

        with self.lock:
            for i in range(self.num_particles):
                px, py, pt = self.particles[i]
                log_w = 0.0

                # Compute all endpoint positions for this particle
                angles = pt + self.lidar_yaw_offset + ray_angles
                hx = px + ray_ranges * np.cos(angles)
                hy = py + ray_ranges * np.sin(angles)

                # Convert to grid coordinates
                mx = ((hx - ox) / res).astype(int)
                my = ((hy - oy) / res).astype(int)

                for j in range(len(indices)):
                    gx = mx[j]
                    gy = my[j]

                    if 0 <= gx < w and 0 <= gy < h:
                        d = self.distance_map[gy, gx]
                    else:
                        d = sigma * 3.0  # Penalty for out of bounds

                    # Mixture model: hit + random
                    p_hit = math.exp(-(d * d) / sigma_sq_2)
                    p_rand = 1.0 / max_range
                    p = z_hit * p_hit + z_rand * p_rand

                    if p > 0:
                        log_w += math.log(p)
                    else:
                        log_w += -100.0

                self.weights[i] = math.exp(min(log_w, 500))

            # Normalize weights
            total = np.sum(self.weights)
            if total > 0:
                self.weights /= total
            else:
                self.get_logger().warn('All particle weights zero — resetting uniform')
                self.weights = np.ones(self.num_particles) / self.num_particles

            # Debug: log weight distribution
            best_idx = np.argmax(self.weights)
            best_p = self.particles[best_idx]
            estimate_x = np.average(self.particles[:, 0], weights=self.weights)
            estimate_y = np.average(self.particles[:, 1], weights=self.weights)
            sin_sum = np.average(np.sin(self.particles[:, 2]), weights=self.weights)
            cos_sum = np.average(np.cos(self.particles[:, 2]), weights=self.weights)
            estimate_theta = math.atan2(sin_sum, cos_sum)
            self.get_logger().info(
                f'SENSOR: best=({best_p[0]:.2f},{best_p[1]:.2f},{math.degrees(best_p[2]):.1f}deg) '
                f'mean=({estimate_x:.2f},{estimate_y:.2f},{math.degrees(estimate_theta):.1f}deg) '
                f'max_w={self.weights[best_idx]:.4f}'
            )

            # Only resample when effective sample size drops below threshold
            n_eff = 1.0 / np.sum(self.weights ** 2)
            if n_eff < self.num_particles * 0.5:
                self.resample()
                self.get_logger().info(f'Resampled (n_eff={n_eff:.0f})', throttle_duration_sec=2.0)

    def _sensor_update_ray_cast(self, scan_msg):
        """
        Ray casting model — fallback when scipy is not available.
        Slower but works without precomputed distance transform.
        """
        ranges = np.array(scan_msg.ranges)
        angle_min = scan_msg.angle_min
        angle_inc = scan_msg.angle_increment

        step = self.scan_subsample
        sigma_sq_2 = 2.0 * self.sigma_hit * self.sigma_hit
        max_range = scan_msg.range_max

        indices = []
        for idx in range(0, len(ranges), step):
            r = ranges[idx]
            if np.isfinite(r) and scan_msg.range_min < r < max_range:
                indices.append(idx)

        if len(indices) == 0:
            return

        with self.lock:
            for i in range(self.num_particles):
                px, py, ptheta = self.particles[i]
                log_weight = 0.0

                for idx in indices:
                    r = ranges[idx]
                    ray_angle = ptheta + self.lidar_yaw_offset + angle_min + idx * angle_inc

                    expected = self._ray_cast(px, py, ray_angle, max_range)

                    diff = r - expected
                    log_weight += -(diff * diff) / sigma_sq_2

                self.weights[i] = math.exp(min(log_weight, 500))

            # Normalize
            total = np.sum(self.weights)
            if total > 0:
                self.weights /= total
            else:
                self.weights = np.ones(self.num_particles) / self.num_particles

            # Only resample when effective sample size drops below threshold
            n_eff = 1.0 / np.sum(self.weights ** 2)
            if n_eff < self.num_particles * 0.5:
                self.resample()
                self.get_logger().info(f'Resampled (n_eff={n_eff:.0f})', throttle_duration_sec=2.0)

    def _ray_cast(self, x, y, angle, max_range):
        """Cast a single ray on the occupancy grid. Returns distance to obstacle."""
        res = self.map_info.resolution
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        h, w = self.map_data.shape

        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        step_size = res * 0.5
        d = 0.0

        while d < max_range:
            wx = x + d * cos_a
            wy = y + d * sin_a

            mx = int((wx - origin_x) / res)
            my = int((wy - origin_y) / res)

            if mx < 0 or my < 0 or my >= h or mx >= w:
                return max_range

            if self.map_data[my, mx] > 50:
                return d

            d += step_size

        return max_range

    # =================================================================
    # RESAMPLING
    # =================================================================

    def resample(self):
        """Low variance resampling with random particle injection near best estimate."""
        N = self.num_particles
        new_particles = np.zeros_like(self.particles)

        n_random = max(1, int(self.random_particle_pct * N))
        n_resample = N - n_random

        # --- Low variance resampling ---
        r = np.random.uniform(0, 1.0 / n_resample) if n_resample > 0 else 0
        c = self.weights[0]
        j = 0

        for i in range(n_resample):
            u = r + i * (1.0 / n_resample)
            while u > c and j < N - 1:
                j += 1
                c += self.weights[j]
            new_particles[i] = self.particles[j]

        # --- Random particle injection near best estimate ---
        best_idx = np.argmax(self.weights)
        bx, by, bt = self.particles[best_idx]

        for i in range(n_resample, N):
            new_particles[i, 0] = bx + np.random.normal(0, 0.15)
            new_particles[i, 1] = by + np.random.normal(0, 0.15)
            new_particles[i, 2] = bt + np.random.normal(0, 0.1)

        self.particles = new_particles
        self.weights = np.ones(N) / N

    # =================================================================
    # POSE ESTIMATE
    # =================================================================

    def get_estimate(self):
        """Get weighted mean pose estimate from particles."""
        if not self.initialized:
            return None, None

        with self.lock:
            x = np.average(self.particles[:, 0], weights=self.weights)
            y = np.average(self.particles[:, 1], weights=self.weights)

            # Circular mean for angle
            sin_sum = np.average(np.sin(self.particles[:, 2]), weights=self.weights)
            cos_sum = np.average(np.cos(self.particles[:, 2]), weights=self.weights)
            theta = math.atan2(sin_sum, cos_sum)

            cov = np.cov(self.particles.T, aweights=self.weights)

        return (x, y, theta), cov

    # =================================================================
    # MAIN TIMER LOOP
    # =================================================================

    def _publish_identity_tf(self):
        """Publish identity map->odom so the map frame exists in TF tree."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.global_frame
        t.child_frame_id = self.odom_frame
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        msg = TFMessage()
        msg.transforms = [t]
        self.tf_pub_reliable.publish(msg)
        self.tf_pub_best_effort.publish(msg)

    def timer_callback(self):
        """Main update loop — motion update every tick, sensor update when threshold met."""
        # Always publish map->odom TF (identity if not initialized yet)
        if not self.initialized:
            self._publish_identity_tf()

        if not self.map_received:
            self.get_logger().warn('Waiting for map...', throttle_duration_sec=5.0)
            return

        if not self.initialized:
            self.get_logger().warn('Waiting for initial pose (use RViz 2D Pose Estimate)...',
                                   throttle_duration_sec=5.0)
            return

        # --- Read current odometry from TF (odom->base, published by simple_ekf) ---
        if not self.odom_tf_received:
            self.get_logger().warn('Waiting for odom->base TF from simple_ekf...',
                                   throttle_duration_sec=5.0)
            return

        odom_x = self.latest_odom_x
        odom_y = self.latest_odom_y
        odom_theta = self.latest_odom_theta

        # --- Motion update (every tick) ---
        if self.last_odom_x is not None:
            dx = odom_x - self.last_odom_x
            dy = odom_y - self.last_odom_y
            dtheta = odom_theta - self.last_odom_theta
            dtheta = math.atan2(math.sin(dtheta), math.cos(dtheta))

            self.motion_update(dx, dy, dtheta)

        self.last_odom_x = odom_x
        self.last_odom_y = odom_y
        self.last_odom_theta = odom_theta

        # --- Sensor update (only when motion threshold met) ---
        if self.last_update_x is not None:
            d = math.sqrt((odom_x - self.last_update_x) ** 2 +
                          (odom_y - self.last_update_y) ** 2)
            a = abs(math.atan2(
                math.sin(odom_theta - self.last_update_theta),
                math.cos(odom_theta - self.last_update_theta)
            ))

            if d > self.update_min_d or a > self.update_min_a:
                if self.latest_scan is not None:
                    # Skip sensor update during fast turns — trust odometry
                    if a < 0.3:
                        self.sensor_update(self.latest_scan)
                    else:
                        self.get_logger().info(f'Skipping sensor update during turn (a={math.degrees(a):.1f}deg)')
                    self.last_update_x = odom_x
                    self.last_update_y = odom_y
                    self.last_update_theta = odom_theta
                    self.get_logger().info(
                        f'Sensor update (d={d:.3f}m, a={math.degrees(a):.1f}deg)',
                        throttle_duration_sec=2.0
                    )
        else:
            # First time — initialize update tracking
            self.last_update_x = odom_x
            self.last_update_y = odom_y
            self.last_update_theta = odom_theta

        # --- Publish pose estimate ---
        estimate, covariance = self.get_estimate()
        if estimate is None:
            return

        x, y, theta = estimate
        self.publish_pose(x, y, theta, covariance)
        self.publish_particles()
        self.publish_tf(x, y, theta, odom_x, odom_y, odom_theta)

    # =================================================================
    # PUBLISHERS
    # =================================================================

    def publish_pose(self, x, y, theta, covariance):
        """Publish estimated pose."""
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.global_frame

        pose_msg.pose.pose.position.x = float(x)
        pose_msg.pose.pose.position.y = float(y)
        pose_msg.pose.pose.position.z = 0.0
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = float(math.sin(theta / 2.0))
        pose_msg.pose.pose.orientation.w = float(math.cos(theta / 2.0))

        if covariance is not None and covariance.shape == (3, 3):
            pose_msg.pose.covariance[0] = float(covariance[0, 0])    # xx
            pose_msg.pose.covariance[1] = float(covariance[0, 1])    # xy
            pose_msg.pose.covariance[6] = float(covariance[1, 0])    # yx
            pose_msg.pose.covariance[7] = float(covariance[1, 1])    # yy
            pose_msg.pose.covariance[35] = float(covariance[2, 2])   # theta-theta

        self.pose_pub.publish(pose_msg)

    def publish_particles(self):
        """Publish particle cloud for visualization in RViz."""
        with self.lock:
            particle_msg = PoseArray()
            particle_msg.header.stamp = self.get_clock().now().to_msg()
            particle_msg.header.frame_id = self.global_frame

            for p in self.particles:
                pose = Pose()
                pose.position.x = float(p[0])
                pose.position.y = float(p[1])
                pose.position.z = 0.0
                pose.orientation.x = 0.0
                pose.orientation.y = 0.0
                pose.orientation.z = float(math.sin(p[2] / 2.0))
                pose.orientation.w = float(math.cos(p[2] / 2.0))
                particle_msg.poses.append(pose)

        self.particle_pub.publish(particle_msg)

    def publish_tf(self, x, y, theta, odom_x, odom_y, odom_theta):
        """
        Publish map->odom transform (Dashing compatible using TFMessage).

        map->base  = AMCL estimate (x, y, theta)
        odom->base = current odom from simple_ekf
        map->odom  = map->base * inverse(odom->base)
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.global_frame
        t.child_frame_id = self.odom_frame

        diff_theta = theta - odom_theta
        cos_diff = math.cos(diff_theta)
        sin_diff = math.sin(diff_theta)

        t.transform.translation.x = float(x - (odom_x * cos_diff - odom_y * sin_diff))
        t.transform.translation.y = float(y - (odom_x * sin_diff + odom_y * cos_diff))
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = float(math.sin(diff_theta / 2.0))
        t.transform.rotation.w = float(math.cos(diff_theta / 2.0))

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
