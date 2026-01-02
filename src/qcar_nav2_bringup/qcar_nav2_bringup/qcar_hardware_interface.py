#!/usr/bin/env python3
import sys
#
sys.path.insert(0, '/home/nvidia/Core Modules/Python')
sys.path.insert(0, '/home/nvidia/Core Modules/Python/Quanser')

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point32
from std_msgs.msg import Header
import numpy as np
import math
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Import QCar hardware
#
from Quanser.product_QCar import QCar

# Import LIDAR
try:
    from pal.utilities.lidar import Lidar as LIDAR
except ImportError:
    try:
        from hal.utilities.lidar import Lidar as LIDAR
    except ImportError:
        print("ERROR: Could not import LIDAR")
        LIDAR = None


class QCarHardwareInterface(Node):
    def __init__(self):
        super().__init__('qcar_hardware_interface')

        # Declare parameters
        self.declare_parameter('max_speed', 0.5)
        self.declare_parameter('max_steering_angle', 0.5)
        self.max_speed = self.get_parameter('max_speed').value
        self.max_steering = self.get_parameter('max_steering_angle').value

        # Initialize QCar hardware ONCE
        try:
            self.qcar = QCar() #
            self.get_logger().info('✓ QCar hardware initialized')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize QCar: {str(e)}')
            raise

        # Initialize LIDAR ONCE
        self.lidar = None
        if LIDAR is not None:
            try:
                self.lidar = LIDAR()
                self.get_logger().info('✓ LIDAR initialized')
            except Exception as e:
                self.get_logger().warn(f'LIDAR initialization failed: {str(e)}')

        # -----------------------------
        # TF publishers (DUAL QoS)
        # -----------------------------
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

        self.tf_publisher_reliable = self.create_publisher(TFMessage, '/tf', tf_qos_reliable)
        self.tf_publisher_best_effort = self.create_publisher(TFMessage, '/tf', tf_qos_best_effort)

        # Other publishers
        self.scan_publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.pointcloud_publisher = self.create_publisher(PointCloud, '/pointcloud', 10)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)

        # Subscriber for motor commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # ----------------------------------------
        # UPDATED: Hardware Constants & States
        # ----------------------------------------
        self.wheel_radius = 0.033  # meters (Updated from 0.034 per Manual Table 11)
        self.wheel_base = 0.256    # meters
        self.encoder_counts_per_rev = 2880  # quadrature
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_encoder_count = 0
        self.last_time = self.get_clock().now()

        # Motor command state
        self.current_throttle = 0.0
        
        # Steering Logic with Lag
        self.target_steering = 0.0   # The desired steering from cmd_vel (radians)
        self.current_steering = 0.0  # The actual filtered steering sent to motors (radians)
        self.steering_tau = 0.16     # Filter Time Constant (0.16s)
        
        self.last_cmd_time = self.get_clock().now()
        self.cmd_timeout = rclpy.duration.Duration(seconds=1.0)

        # QCar max physical steering angle is approximately ±30 degrees (±0.5236 rad)
        # The QCar API saturates at 0.5 rad
        self.max_physical_steering_angle = 0.5236 
        
        # Computed angular velocity (for odometry feedback)
        self.computed_angular_velocity = 0.0

        # Main control loop (50 Hz -> dt = 0.02s)
        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info('✓ QCar Hardware Interface ready (Nav2-compatible odometry)')
        self.get_logger().info(f'  Wheel Radius: {self.wheel_radius}m (Updated)')
        self.get_logger().info(f'  Steering: Direct Radians + 0.16s Lag Filter')

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = self.get_clock().now()

        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # Throttle command (simple scaling)
        # Note: product_QCar saturates internally at +/- 0.2
        self.current_throttle = np.clip(linear_vel / self.max_speed * 0.2, -0.2, 0.2)

        # ----------------------------------------
        # UPDATED: Direct Radian Calculation
        # ----------------------------------------
        # 1. Compute required steering angle (radians) from bicycle model
        steering_angle_rad = self.compute_steering_from_angular_velocity(linear_vel, angular_vel)
        
        # 2. Clip to hardware limits (0.5 rad is QCar API limit)
        # We do NOT normalize to [-0.5, 0.5] anymore. We send radians.
        self.target_steering = np.clip(steering_angle_rad, -0.5, 0.5)

    def control_loop(self):
        try:
            current_time = self.get_clock().now()
            dt = 0.02 # Fixed timer period

            time_since_cmd = current_time - self.last_cmd_time
            if time_since_cmd > self.cmd_timeout:
                self.current_throttle = 0.0
                self.target_steering = 0.0

            # ----------------------------------------
            # UPDATED: Steering Lag Filter (First Order)
            # ----------------------------------------
            # y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
            # alpha = dt / (tau + dt)
            alpha = dt / (self.steering_tau + dt)
            
            self.current_steering = (alpha * self.target_steering) + ((1.0 - alpha) * self.current_steering)

            # Send to Hardware (Direct Radians)
            mtr_cmd = np.array([self.current_throttle, self.current_steering])
            LEDs = np.zeros(8, dtype=int)
            
            # read_write_std returns (current, voltage, encoder_counts)
            motor_current, battery_voltage, encoder_count = self.qcar.read_write_std(mtr_cmd, LEDs)

            # Update odom + publish TF first
            self.update_odometry(current_time, encoder_count)

            # Then lidar
            if self.lidar is not None:
                try:
                    self.lidar.read()
                    self.publish_laserscan(current_time)
                    self.publish_pointcloud(current_time)
                except Exception as e:
                    self.get_logger().error(f'LIDAR read error: {str(e)}', throttle_duration_sec=5.0)

        except Exception as e:
            self.get_logger().error(f'Control loop error: {str(e)}')

    def compute_steering_from_angular_velocity(self, linear_velocity, desired_angular_velocity):
        """
        Convert desired angular velocity to required steering angle (Radians).
        """
        # Handle stationary rotation or very slow speeds
        if abs(linear_velocity) < 0.05:
            if abs(desired_angular_velocity) > 0.01:
                # Return max steering in direction of rotation (Radians)
                return np.sign(desired_angular_velocity) * 0.5
            else:
                return 0.0
        
        # Normal case: compute steering angle from bicycle model
        # δ = atan((L * ω) / v)
        try:
            steering_angle = math.atan((self.wheel_base * desired_angular_velocity) / linear_velocity)
            return steering_angle
        except:
            return 0.0

    def compute_angular_velocity_from_steering(self, linear_velocity, steering_command_rad):
        """
        Compute angular velocity using bicycle/Ackermann steering model.
        Args:
            steering_command_rad: Steering angle in RADIANS
        """
        # Prevent division by zero and handle small velocities
        if abs(linear_velocity) < 0.001:
            return 0.0
        
        # Bicycle model: ω = (v / L) * tan(δ)
        try:
            angular_velocity = (linear_velocity / self.wheel_base) * math.tan(steering_command_rad)
            # Sanity check limit
            return np.clip(angular_velocity, -4.0, 4.0)
        except:
            return 0.0

    def update_odometry(self, current_time, encoder_count):
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return

        # Compute linear velocity from wheel encoders
        delta_encoder = encoder_count - self.last_encoder_count
        encoder_speed = delta_encoder / dt

        wheel_angular_velocity = (encoder_speed / self.encoder_counts_per_rev) * 2.0 * math.pi
        linear_velocity = wheel_angular_velocity * self.wheel_radius

        # Compute angular velocity from steering using bicycle model
        # Pass the FILTERED current steering (in radians)
        angular_velocity = self.compute_angular_velocity_from_steering(
            linear_velocity, 
            self.current_steering 
        )
        
        # Store for feedback
        self.computed_angular_velocity = angular_velocity

        # Dead reckoning integration
        delta_s = linear_velocity * dt
        delta_theta = angular_velocity * dt

        self.theta += delta_theta
        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        self.x += delta_s * math.cos(self.theta)
        self.y += delta_s * math.sin(self.theta)

        # Publish TF + odom
        self.publish_tf(current_time)
        self.publish_odometry(current_time, linear_velocity, angular_velocity)

        self.last_encoder_count = encoder_count
        self.last_time = current_time

    def publish_laserscan(self, current_time):
        scan_msg = LaserScan()
        scan_msg.header = Header()
        scan_msg.header.stamp = current_time.to_msg()
        scan_msg.header.frame_id = 'lidar'

        scan_msg.range_min = 0.15
        scan_msg.range_max = 12.0

        ranges = np.array(self.lidar.distances).flatten().astype(np.float32)

        if ranges.size > 0 and np.nanmax(ranges) > 50.0:
            ranges = ranges / 1000.0

        ranges = np.where(
            (~np.isfinite(ranges)) |
            (ranges < scan_msg.range_min) |
            (ranges > scan_msg.range_max),
            np.inf,
            ranges
        )

        n = int(ranges.size)
        if n <= 1:
            return

        scan_msg.angle_min = 0.0
        scan_msg.angle_max = 2.0 * math.pi
        scan_msg.angle_increment = (2.0 * math.pi) / n
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.033
        scan_msg.ranges = ranges.tolist()
        scan_msg.intensities = []

        self.scan_publisher.publish(scan_msg)

    def publish_pointcloud(self, current_time):
        pc_msg = PointCloud()
        pc_msg.header = Header()
        pc_msg.header.stamp = current_time.to_msg()
        pc_msg.header.frame_id = 'lidar'

        ranges = np.array(self.lidar.distances).flatten().astype(np.float32)
        angles = np.array(self.lidar.angles).flatten().astype(np.float32) if hasattr(self.lidar, 'angles') else None

        if ranges.size == 0 or angles is None or angles.size != ranges.size:
            return

        if np.nanmax(ranges) > 50.0:
            ranges = ranges / 1000.0

        points = []
        for r, a in zip(ranges, angles):
            if not np.isfinite(r) or r < 0.15 or r > 12.0:
                continue
            pt = Point32()
            pt.x = float(r * math.cos(float(a)))
            pt.y = float(r * math.sin(float(a)))
            pt.z = 0.0
            points.append(pt)

        pc_msg.points = points
        self.pointcloud_publisher.publish(pc_msg)

    def publish_odometry(self, current_time, linear_vel, angular_vel):
        odom_msg = Odometry()
        odom_msg.header = Header()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        quat = self.euler_to_quaternion(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        odom_msg.twist.twist.linear.x = linear_vel
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = angular_vel

        # Covariance matrices
        odom_msg.pose.covariance = [0.0] * 36
        odom_msg.pose.covariance[0] = 0.01   # x variance
        odom_msg.pose.covariance[7] = 0.01   # y variance
        odom_msg.pose.covariance[35] = 0.05  # yaw variance

        odom_msg.twist.covariance = [0.0] * 36
        odom_msg.twist.covariance[0] = 0.01   # linear x
        odom_msg.twist.covariance[35] = 0.05  # angular z

        self.odom_publisher.publish(odom_msg)

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

    def publish_tf(self, current_time):
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base'

        t.transform.translation.x = float(self.x)
        t.transform.translation.y = float(self.y)
        t.transform.translation.z = 0.0

        quat = self.euler_to_quaternion(0, 0, self.theta)
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])

        msg = TFMessage()
        msg.transforms = [t]

        self.tf_publisher_reliable.publish(msg)
        self.tf_publisher_best_effort.publish(msg)

    def destroy_node(self):
        try:
            mtr_cmd = np.array([0.0, 0.0])
            LEDs = np.zeros(8, dtype=int)
            self.qcar.read_write_std(mtr_cmd, LEDs)
            self.qcar.terminate()
            self.get_logger().info('QCar hardware terminated')
        except Exception as e:
            self.get_logger().error(f'Error terminating QCar: {str(e)}')

        if self.lidar is not None:
            try:
                self.lidar.terminate()
                self.get_logger().info('LIDAR terminated')
            except Exception as e:
                self.get_logger().error(f'Error terminating LIDAR: {str(e)}')

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = QCarHardwareInterface()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nShutdown requested')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
