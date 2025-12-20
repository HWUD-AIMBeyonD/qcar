#!/usr/bin/env python3

import sys
import os

# Add path to Core Modules
sys.path.insert(0, '/home/nvidia/Core Modules/Python/Quanser')

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Header
import numpy as np
import math

# Import LIDAR from pal package
try:
    from pal.utilities.lidar import Lidar as LIDAR
except ImportError:
    try:
        from hal.utilities.lidar import Lidar as LIDAR
    except ImportError:
        print("ERROR: Could not import LIDAR")
        sys.exit(1)


class QCarLidarNode(Node):
    def __init__(self):
        super().__init__('qcar_lidar_node')
        
        try:
            # Initialize LIDAR with NO parameters
            self.lidar = LIDAR()
            self.get_logger().info('LIDAR initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize LIDAR: {str(e)}')
            raise
        
        self.scan_publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.pointcloud_publisher = self.create_publisher(PointCloud, '/pointcloud', 10)
        
        self.timer = self.create_timer(0.033, self.timer_callback)
        
        self.get_logger().info('QCar LIDAR Node started')

    def timer_callback(self):
        try:
            self.lidar.read()
            self.publish_laserscan()
            self.publish_pointcloud()
        except Exception as e:
            self.get_logger().error(f'Error in timer callback: {str(e)}')

    def publish_laserscan(self):
        scan_msg = LaserScan()
        scan_msg.header = Header()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'lidar'
        scan_msg.angle_min = 0.0
        scan_msg.angle_max = 2.0 * math.pi
        scan_msg.angle_increment = (2.0 * math.pi) / 720
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.033
        scan_msg.range_min = 0.12
        scan_msg.range_max = 10.0
        scan_msg.ranges = self.lidar.distances.flatten().tolist()
        scan_msg.intensities = []
        self.scan_publisher.publish(scan_msg)

    def publish_pointcloud(self):
        pc_msg = PointCloud()
        pc_msg.header = Header()
        pc_msg.header.stamp = self.get_clock().now().to_msg()
        pc_msg.header.frame_id = 'lidar'
        ranges = self.lidar.distances.flatten()
        angles = self.lidar.angles
        points = []
        for r, a in zip(ranges, angles):
            if r > 0.12 and r < 10.0:
                pt = Point32()
                pt.x = float(r * math.cos(a))
                pt.y = float(r * math.sin(a))
                pt.z = 0.0
                points.append(pt)
        pc_msg.points = points
        self.pointcloud_publisher.publish(pc_msg)

    def destroy_node(self):
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
        node = QCarLidarNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nShutdown requested by user')
    except Exception as e:
        print(f'Error: {str(e)}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

