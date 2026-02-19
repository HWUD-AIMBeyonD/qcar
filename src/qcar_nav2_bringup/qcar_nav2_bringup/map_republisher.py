#!/usr/bin/env python3
"""
Simple map republisher to work around QoS mismatch in ROS2 Dashing.
Subscribes to /map_raw and republishes to /map with AMCL-compatible QoS.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

class MapRepublisher(Node):
    def __init__(self):
        super().__init__('map_republisher')
        
        # Subscribe with RELIABLE + TRANSIENT_LOCAL (matches map_server's publisher)
        sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publish with RELIABLE + TRANSIENT_LOCAL (what AMCL expects)
        pub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.map_data = None
        
        self.subscriber = self.create_subscription(
            OccupancyGrid,
            '/map_raw',
            self.map_callback,
            sub_qos
        )
        
        self.publisher = self.create_publisher(OccupancyGrid, '/map', pub_qos)
        
        # Republish periodically (backup in case of timing issues)
        self.timer = self.create_timer(2.0, self.republish)
        
        self.get_logger().info('Map republisher started (map_raw -> map)')
    
    def map_callback(self, msg):
        self.map_data = msg
        self.get_logger().info(f'Received map: {msg.info.width}x{msg.info.height}')
        # Publish immediately when received
        self.publisher.publish(self.map_data)
    
    def republish(self):
        if self.map_data is not None:
            self.publisher.publish(self.map_data)
            self.get_logger().debug('Republished map')
        else:
            self.get_logger().warn('No map data yet to republish')

def main(args=None):
    rclpy.init(args=args)
    node = MapRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
