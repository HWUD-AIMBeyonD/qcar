#!/usr/bin/env python3
import sys
#
sys.path.insert(0, '/home/nvidia/Core Modules/Python')
sys.path.insert(0, '/home/nvidia/Core Modules/Python/Quanser')

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point32
from std_msgs.msg import Header
import numpy as np
import math
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Import QCar hardware
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
            self.qcar = QCar() 
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
        self.odom_publisher = self.create_publisher(Odometry, '/odom_raw', 10)
        
        # --- IMU PUBLISHER ---
        self.imu_publisher = self.create_publisher(Imu, '/imu', 10)

        # Subscriber for motor commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # ----------------------------------------
        # Hardware Constants & States
        # ----------------------------------------
        self.wheel_radius = 0.033  
        self.wheel_base = 0.256     
        self.encoder_counts_per_rev = 28800  
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_encoder_count = 0
        self.last_time = self.get_clock().now()

        # Motor command state
        self.current_throttle = 0.0
        self.target_steering = 0.0  
        self.current_steering = 0.0 
        self.steering_tau = 0.16     
        
        self.last_cmd_time = self.get_clock().now()
        self.cmd_timeout = rclpy.duration.Duration(seconds=1.0)
        
        self.computed_angular_velocity = 0.0

        # Main control loop (50 Hz -> dt = 0.02s)
        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info('✓ QCar Hardware Interface ready')
        self.get_logger().info('  /odom_raw - Wheel encoder odometry')
        self.get_logger().info('  /imu - IMU data')

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = self.get_clock().now()

        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # Throttle command
        self.current_throttle = np.clip(linear_vel / self.max_speed * 0.2, -0.2, 0.2)

        # Steering command
        steering_angle_rad = self.compute_steering_from_angular_velocity(linear_vel, angular_vel)
        self.target_steering = np.clip(steering_angle_rad, -0.5, 0.5)

    def control_loop(self):
        try:
            current_time = self.get_clock().now()
            dt = 0.02 

            time_since_cmd = current_time - self.last_cmd_time
            if time_since_cmd > self.cmd_timeout:
                self.current_throttle = 0.0
                self.target_steering = 0.0

            # Steering Lag Filter
            alpha = dt / (self.steering_tau + dt)
            self.current_steering = (alpha * self.target_steering) + ((1.0 - alpha) * self.current_steering)

            # Send to Hardware
            mtr_cmd = np.array([self.current_throttle, self.current_steering])
            LEDs = np.zeros(8, dtype=int)
            
            # Read Basic Sensors (Current, Voltage, Encoders)
            motor_current, battery_voltage, encoder_count = self.qcar.read_write_std(mtr_cmd, LEDs)

            # --- Read IMU Data (with workaround for Core Modules bug) ---
            try:
                # Call read_IMU to populate the buffer
                self.qcar.read_IMU()

                # Extract data directly from buffer (bypassing buggy return)
                gyro = self.qcar.read_other_buffer_IMU[0:3]   # Channels 3000-3002
                accel = self.qcar.read_other_buffer_IMU[3:6]  # Channels 4000-4002

                self.publish_imu(current_time, gyro, accel)
            except Exception as e:
                # Occasional read errors shouldn't kill the node
                pass

            # Update odom + publish TF
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

    def publish_imu(self, current_time, gyro, accel):
        imu_msg = Imu()
        imu_msg.header.stamp = current_time.to_msg()
        imu_msg.header.frame_id = 'imu_link' # Matches URDF

        # Orientation: Unknown (Covariance -1). Cartographer will use Gyro instead.
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 1.0
        imu_msg.orientation_covariance[0] = -1

        # Angular Velocity (Gyro) - rad/s
        imu_msg.angular_velocity.x = float(gyro[0])
        imu_msg.angular_velocity.y = float(gyro[1])
        imu_msg.angular_velocity.z = float(gyro[2])
        imu_msg.angular_velocity_covariance[0] = 0.01
        imu_msg.angular_velocity_covariance[4] = 0.01
        imu_msg.angular_velocity_covariance[8] = 0.01

        # Linear Acceleration (Accel) - m/s^2
        imu_msg.linear_acceleration.x = float(accel[0])
        imu_msg.linear_acceleration.y = float(accel[1])
        imu_msg.linear_acceleration.z = float(accel[2])
        imu_msg.linear_acceleration_covariance[0] = 0.1
        imu_msg.linear_acceleration_covariance[4] = 0.1
        imu_msg.linear_acceleration_covariance[8] = 0.1

        self.imu_publisher.publish(imu_msg)

    def compute_steering_from_angular_velocity(self, linear_velocity, desired_angular_velocity):
        if abs(linear_velocity) < 0.05:
            if abs(desired_angular_velocity) > 0.01:
                return np.sign(desired_angular_velocity) * 0.5
            else:
                return 0.0
        try:
            steering_angle = math.atan((self.wheel_base * desired_angular_velocity) / linear_velocity)
            return steering_angle
        except:
            return 0.0

    def compute_angular_velocity_from_steering(self, linear_velocity, steering_command_rad):
        if abs(linear_velocity) < 0.001:
            return 0.0
        try:
            angular_velocity = (linear_velocity / self.wheel_base) * math.tan(steering_command_rad)
            return np.clip(angular_velocity, -4.0, 4.0)
        except:
            return 0.0

    def update_odometry(self, current_time, encoder_count):
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return

        delta_encoder = encoder_count - self.last_encoder_count
        encoder_speed = delta_encoder / dt

        wheel_angular_velocity = (encoder_speed / self.encoder_counts_per_rev) * 2.0 * math.pi
        linear_velocity = wheel_angular_velocity * self.wheel_radius

        angular_velocity = self.compute_angular_velocity_from_steering(
            linear_velocity,
            self.current_steering
        )

        self.computed_angular_velocity = angular_velocity

        delta_s = linear_velocity * dt
        delta_theta = angular_velocity * dt

        self.theta += delta_theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        self.x += delta_s * math.cos(self.theta)
        self.y += delta_s * math.sin(self.theta)

        # TF odom -> base is published by simple_ekf node
        # self.publish_tf(current_time)

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

        angles = np.array(self.lidar.angles).flatten().astype(np.float32)

        n = int(ranges.size)
        if n <= 1 or angles.size != n:
            return
        
        scan_msg.angle_min = float(angles[-1])
        scan_msg.angle_max = float(angles[0])
        scan_msg.angle_increment = float((angles[0] - angles[-1]) / (n - 1))
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

        # Provide position (from encoder integration)
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        quat = self.euler_to_quaternion(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        # Provide velocity
        odom_msg.twist.twist.linear.x = linear_vel
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = angular_vel

        # Covariance - trust x,y but NOT yaw (IMU is better for yaw)
        odom_msg.pose.covariance = [0.0] * 36
        odom_msg.pose.covariance[0] = 0.1    # x
        odom_msg.pose.covariance[7] = 0.1    # y
        odom_msg.pose.covariance[35] = 1.0   # yaw - higher = less trust

        odom_msg.twist.covariance = [0.0] * 36
        odom_msg.twist.covariance[0] = 0.1    # vx
        odom_msg.twist.covariance[35] = 1.0   # vyaw - higher = less trust

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

