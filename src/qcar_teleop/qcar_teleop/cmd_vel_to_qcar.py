#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import numpy as np

# Use YOUR actual working path from the manual_control.py script
sys.path.insert(0, '/home/nvidia/Core Modules/Python')
from Quanser.product_QCar import QCar  # Match your working import

class CmdVelToQCar(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_qcar')
        
        # Declare parameters
        self.declare_parameter('max_speed', 0.5)
        self.declare_parameter('max_steering_angle', 0.5)
        
        self.max_speed = self.get_parameter('max_speed').value
        self.max_steering = self.get_parameter('max_steering_angle').value
        
        # Initialize QCar (same as your working scripts)
        try:
            self.qcar = QCar()
            self.get_logger().info('QCar initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize QCar: {str(e)}')
            raise
        
        # Subscribe to cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Safety timeout
        self.last_cmd_time = self.get_clock().now()
        self.timeout_duration = rclpy.duration.Duration(seconds=1.0)
        self.timer = self.create_timer(0.1, self.safety_check)
        
        self.get_logger().info('Listening to /cmd_vel')

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = self.get_clock().now()
        
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        
        # Scale to QCar limits (matching your working code)
        throttle = np.clip(linear_vel / self.max_speed * 0.2, -0.2, 0.2)
        steering = np.clip(angular_vel * 0.5, -0.5, 0.5)
        
        mtr_cmd = np.array([throttle, steering])
        
        try:
            self.qcar.write_mtrs(mtr_cmd)
        except Exception as e:
            self.get_logger().error(f'Motor write failed: {str(e)}')

    def safety_check(self):
        time_since_last_cmd = self.get_clock().now() - self.last_cmd_time
        if time_since_last_cmd > self.timeout_duration:
            mtr_cmd = np.array([0.0, 0.0])
            try:
                self.qcar.write_mtrs(mtr_cmd)
            except:
                pass

    def destroy_node(self):
        mtr_cmd = np.array([0.0, 0.0])
        try:
            self.qcar.write_mtrs(mtr_cmd)
            self.qcar.terminate()
        except:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToQCar()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

