#!/usr/bin/env python3
"""
ROS 2 (Dashing) port of Quanser's ROS 1 RGBD node.

Publishes the Intel RealSense colour and depth streams from the QCar:
  /qcar/rgbd_color             sensor_msgs/Image            bgr8
  /qcar/rgbd_color/compressed  sensor_msgs/CompressedImage  jpeg (compress:=True,
                                                            replaces the raw topic)
  /qcar/rgbd_depth             sensor_msgs/Image            32FC1 (metres)

Images are built directly from numpy arrays rather than through cv_bridge,
so the node has no cv_bridge / OpenCV-version dependency on the Jetson.
OpenCV is only imported for compress:=True (JPEG encoding).

Parameters:
  width, height    RGB resolution passed to Camera3D (default 1280x720,
                   same as the original Quanser node)
  publish_rate     Hz cap on publishing; 0 = publish every frame
  publish_depth    set False to skip reading and publishing depth
  compress         publish JPEG CompressedImage instead of the raw colour image.
                   Raw 1280x720 bgr8 is ~2.7 MB/frame -- too much for rosbag2's
                   sqlite writer on the Jetson. Foxglove shows either.
  jpeg_quality     0-100, used with compress
  color_frame_id   header.frame_id for the colour image (camera_rgb exists in
                   the URDF, so Foxglove's 3D panel can place it)
  depth_frame_id   header.frame_id for the depth image

Dashing has no --ros-args -p; pass parameters with a YAML file:
  python3 rgbd_camera_node.py __params:=../config/rgbd_camera.yaml
(run_qcar_camera.sh does this.)
"""

import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage

# qcar.q_essential is the ROS 1 catkin package path; on the ROS 2 side the
# Quanser modules are loaded from Core Modules, same as qcar_hardware_interface.
try:
    from qcar.q_essential import Camera3D
except ImportError:
    sys.path.insert(0, '/home/nvidia/Core Modules/Python')
    sys.path.insert(0, '/home/nvidia/Core Modules/Python/Quanser')
    from Quanser.q_essential import Camera3D


def to_image_msg(img, encoding, frame_id, stamp):
    """Pack a numpy image into a sensor_msgs/Image without cv_bridge."""
    img = np.ascontiguousarray(img)
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = int(img.shape[0])
    msg.width = int(img.shape[1])
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = int(img.strides[0])  # bytes per row
    msg.data = img.tobytes()
    return msg


class RGBDNode(Node):
    def __init__(self):
        super().__init__('rgbd_node')

        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('publish_rate', 0.0)
        self.declare_parameter('publish_depth', True)
        self.declare_parameter('compress', False)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('color_frame_id', 'camera_rgb')
        self.declare_parameter('depth_frame_id', 'camera_rgb')

        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        rate = float(self.get_parameter('publish_rate').value)
        self.period = 1.0 / rate if rate > 0.0 else 0.0
        self.publish_depth = bool(self.get_parameter('publish_depth').value)
        self.compress = bool(self.get_parameter('compress').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.color_frame_id = self.get_parameter('color_frame_id').value
        self.depth_frame_id = self.get_parameter('depth_frame_id').value

        self.cv2 = None
        if self.compress:
            try:
                import cv2
                self.cv2 = cv2
            except ImportError:
                self.get_logger().warn(
                    'compress:=True but OpenCV is not importable -- publishing raw')
                self.compress = False

        if self.compress:
            self.color_pub = self.create_publisher(
                CompressedImage, '/qcar/rgbd_color/compressed', 10)
        else:
            self.color_pub = self.create_publisher(Image, '/qcar/rgbd_color', 10)
        self.depth_pub = None
        if self.publish_depth:
            self.depth_pub = self.create_publisher(Image, '/qcar/rgbd_depth', 10)

        mode = 'RGB&DEPTH' if self.publish_depth else 'RGB'
        self.rgbd = Camera3D(mode=mode,
                             frame_width_RGB=width,
                             frame_height_RGB=height)
        self.get_logger().info(
            'RealSense started: mode=%s, %dx%d, publish_rate=%s, compress=%s'
            % (mode, width, height, rate if rate > 0.0 else 'unlimited',
               self.compress))

    def to_compressed_msg(self, img, frame_id, stamp):
        ok, buf = self.cv2.imencode(
            '.jpg', img, [int(self.cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            return None
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.format = 'jpeg'
        msg.data = buf.tobytes()
        return msg

    def run(self):
        last_pub = 0.0
        try:
            while rclpy.ok():
                # Always read so the camera buffer keeps draining.
                self.rgbd.read_RGB()
                if self.publish_depth:
                    self.rgbd.read_depth(dataMode='m')

                now = time.monotonic()
                if self.period and (now - last_pub) < self.period:
                    rclpy.spin_once(self, timeout_sec=0)
                    continue
                last_pub = now

                stamp = self.get_clock().now().to_msg()

                color = self.rgbd.image_buffer_RGB.astype(np.uint8, copy=False)
                if self.compress:
                    msg = self.to_compressed_msg(color, self.color_frame_id, stamp)
                    if msg is not None:
                        self.color_pub.publish(msg)
                else:
                    self.color_pub.publish(
                        to_image_msg(color, 'bgr8', self.color_frame_id, stamp))

                if self.publish_depth:
                    depth = self.rgbd.image_buffer_depth_m.astype(
                        np.float32, copy=False)
                    self.depth_pub.publish(
                        to_image_msg(depth, '32FC1', self.depth_frame_id, stamp))

                rclpy.spin_once(self, timeout_sec=0)
        finally:
            self.rgbd.terminate()
            self.get_logger().info('RealSense terminated.')


def main(args=None):
    rclpy.init(args=args)
    node = RGBDNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
