#!/usr/bin/env python3
"""
Direct map publisher that bypasses map_server entirely.
Loads map from yaml/pgm and publishes with exact QoS that AMCL expects.
"""
import os
import yaml
import numpy as np
from PIL import Image

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

class DirectMapPublisher(Node):
    def __init__(self):
        super().__init__('direct_map_publisher')
        
        self.declare_parameter('yaml_filename', '')
        self.declare_parameter('frame_id', 'map')
        
        yaml_file = self.get_parameter('yaml_filename').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # QoS that Dashing AMCL expects - try multiple combinations
        pub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.publisher = self.create_publisher(OccupancyGrid, '/map', pub_qos)
        
        if yaml_file:
            self.map_msg = self.load_map(yaml_file)
            if self.map_msg:
                # Publish immediately
                self.publisher.publish(self.map_msg)
                self.get_logger().info(f'Published map: {self.map_msg.info.width}x{self.map_msg.info.height}')
                
                # Keep republishing every 2 seconds
                self.timer = self.create_timer(0.5, self.republish_map)
        else:
            self.get_logger().error('No yaml_filename provided!')
    
    def load_map(self, yaml_file):
        try:
            with open(yaml_file, 'r') as f:
                map_metadata = yaml.safe_load(f)
            
            yaml_dir = os.path.dirname(yaml_file)
            image_file = os.path.join(yaml_dir, map_metadata['image'])
            
            self.get_logger().info(f'Loading map from {image_file}')
            
            img = Image.open(image_file)
            img_array = np.array(img, dtype=np.uint8)
            
            resolution = float(map_metadata['resolution'])
            origin = map_metadata['origin']
            negate = int(map_metadata.get('negate', 0))
            occupied_thresh = float(map_metadata.get('occupied_thresh', 0.65))
            free_thresh = float(map_metadata.get('free_thresh', 0.196))
            
            # Convert image to occupancy values
            # PGM: 0=black(occupied), 255=white(free), 205=gray(unknown)
            height, width = img_array.shape[:2]
            if len(img_array.shape) > 2:
                img_array = img_array[:, :, 0]  # Take first channel if color
            
            occupancy_data = np.zeros(height * width, dtype=np.int8)
            
            for i in range(height):
                for j in range(width):
                    pixel = img_array[i, j]
                    
                    if negate:
                        pixel = 255 - pixel
                    
                    # Normalize to 0-1
                    normalized = (255 - pixel) / 255.0
                    
                    # Map row is flipped (image y=0 is top, map y=0 is bottom)
                    map_idx = (height - 1 - i) * width + j
                    
                    if normalized >= occupied_thresh:
                        occupancy_data[map_idx] = 100  # Occupied
                    elif normalized <= free_thresh:
                        occupancy_data[map_idx] = 0    # Free
                    else:
                        occupancy_data[map_idx] = -1   # Unknown
            
            # Build message
            msg = OccupancyGrid()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            
            msg.info.resolution = resolution
            msg.info.width = width
            msg.info.height = height
            msg.info.origin.position.x = float(origin[0])
            msg.info.origin.position.y = float(origin[1])
            msg.info.origin.position.z = 0.0
            
            # Handle yaw in origin
            if len(origin) > 2:
                yaw = float(origin[2])
                msg.info.origin.orientation.z = float(np.sin(yaw / 2.0))
                msg.info.origin.orientation.w = float(np.cos(yaw / 2.0))
            else:
                msg.info.origin.orientation.w = 1.0
            
            msg.data = occupancy_data.tolist()
            
            self.get_logger().info(f'Map loaded: {width}x{height}, resolution={resolution}')
            return msg
            
        except Exception as e:
            self.get_logger().error(f'Failed to load map: {e}')
            import traceback
            traceback.print_exc()
            return None
    
    def republish_map(self):
        if self.map_msg:
            self.map_msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(self.map_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DirectMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
