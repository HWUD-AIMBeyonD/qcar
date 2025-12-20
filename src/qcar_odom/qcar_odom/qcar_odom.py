#!/usr/bin/env python3
import sys
sys.path.insert(0, '/home/nvidia/Core Modules/Python')

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Header

from geometry_msgs.msg import TransformStamped          # NEW
from tf2_msgs.msg import TFMessage                      # NEW

import numpy as np
import math

from Quanser.product_QCar import QCar


class QCarOdometryNode(Node):
    def __init__(self):
        super().__init__('qcar_odometry_node')

        try:
            self.qcar = QCar()
            self.get_logger().info('QCar hardware initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize QCar: {str(e)}')
            raise

        self.wheel_radius = 0.034
        self.wheel_base = 0.256
        self.encoder_counts_per_rev = 2880

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_encoder_count = 0
        self.last_time = self.get_clock().now()

        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)

        # NEW: publish TF messages directly on /tf
        self.tf_publisher = self.create_publisher(TFMessage, '/tf', 10)

        self.timer = self.create_timer(0.02, self.timer_callback)
        self.get_logger().info('QCar Odometry Node started (publishing /odom and /tf odom->base)')

    def timer_callback(self):
        try:
            current_time = self.get_clock().now()

            mtr_cmd = np.array([0.0, 0.0])
            LEDs = np.zeros(8, dtype=int)
            motor_current, battery_voltage, encoder_count = self.qcar.read_write_std(mtr_cmd, LEDs)

            angular_velocity = 0.0  # placeholder

            dt = (current_time - self.last_time).nanoseconds / 1e9
            if dt > 0:
                delta_encoder = encoder_count - self.last_encoder_count
                encoder_speed = delta_encoder / dt

                wheel_angular_velocity = (encoder_speed / self.encoder_counts_per_rev) * 2.0 * math.pi
                linear_velocity = wheel_angular_velocity * self.wheel_radius

                delta_s = linear_velocity * dt
                delta_theta = angular_velocity * dt

                self.theta += delta_theta
                self.x += delta_s * math.cos(self.theta)
                self.y += delta_s * math.sin(self.theta)

                self.publish_odometry(current_time, linear_velocity, angular_velocity)
                self.publish_tf(current_time)  # NEW

                self.last_encoder_count = encoder_count
                self.last_time = current_time

        except Exception as e:
            self.get_logger().error(f'Error in timer callback: {str(e)}')

    def publish_odometry(self, current_time, linear_vel, angular_vel):
        odom_msg = Odometry()

        odom_msg.header = Header()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base'

        odom_msg.pose.pose.position.x = float(self.x)
        odom_msg.pose.pose.position.y = float(self.y)
        odom_msg.pose.pose.position.z = 0.0

        qx, qy, qz, qw = self.euler_to_quaternion(0.0, 0.0, self.theta)
        odom_msg.pose.pose.orientation.x = float(qx)
        odom_msg.pose.pose.orientation.y = float(qy)
        odom_msg.pose.pose.orientation.z = float(qz)
        odom_msg.pose.pose.orientation.w = float(qw)

        odom_msg.twist.twist.linear.x = float(linear_vel)
        odom_msg.twist.twist.angular.z = float(angular_vel)

        odom_msg.pose.covariance = [0.0] * 36
        odom_msg.pose.covariance[0] = 0.01
        odom_msg.pose.covariance[7] = 0.01
        odom_msg.pose.covariance[35] = 0.05

        odom_msg.twist.covariance = [0.0] * 36
        odom_msg.twist.covariance[0] = 0.01
        odom_msg.twist.covariance[35] = 0.05

        self.odom_publisher.publish(odom_msg)

    def publish_tf(self, current_time):
        """Publish TF: odom -> base (Dashing-friendly: publish TFMessage to /tf)."""
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base'

        t.transform.translation.x = float(self.x)
        t.transform.translation.y = float(self.y)
        t.transform.translation.z = 0.0

        qx, qy, qz, qw = self.euler_to_quaternion(0.0, 0.0, self.theta)
        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)

        msg = TFMessage()
        msg.transforms = [t]
        self.tf_publisher.publish(msg)

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

    def destroy_node(self):
        try:
            self.qcar.terminate()
            self.get_logger().info('QCar hardware terminated')
        except Exception as e:
            self.get_logger().error(f'Error terminating QCar: {str(e)}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = QCarOdometryNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nShutdown requested by user')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

