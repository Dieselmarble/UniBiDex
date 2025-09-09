#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import yaml
import argparse
import sys

class MultiCameraPublisher(Node):
    """
    ROS2 node to publish multiple camera streams:
    - Cameras are configured through a YAML configuration file passed as a command-line argument.
    """
    def __init__(self, config):
        super().__init__('multi_camera_publisher')

        # Retrieve camera devices and fps from config
        camera_devices = config.get('camera_devices', {})
        fps = config.get('fps', 30)

        # Local dictionaries to store publishers and captures
        publishers = {}
        captures = {}
        self.bridge = CvBridge()

        # Dynamically create publishers and capture objects based on the config
        for name, device in camera_devices.items():
            publishers[name] = self.create_publisher(Image, f'/camera/{name}/image_raw', 10)
            captures[name] = self.open_camera(device)

        # Check openings
        for name, cap in captures.items():
            if not cap.isOpened():
                self.get_logger().error(f'Unable to open {name} camera')

        # Configure resolution, format, fps for available cameras
        self.configure_cameras(captures, fps)

        # Timer for publishing
        period = 1.0 / fps
        self.timer = self.create_timer(period, lambda: self.publish_all(publishers, captures))
        self.get_logger().info(f'MultiCameraPublisher started: {fps}Hz')

    def open_camera(self, device):
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not cap.isOpened():
            self.get_logger().warning(f'Failed CAP_V4L2 for {device}, trying ANY')
            cap.open(device, cv2.CAP_ANY)
        return cap

    def configure_cameras(self, captures, fps):
        for name, cap in captures.items():
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                cap.set(cv2.CAP_PROP_FPS, fps)
                self.get_logger().info(f'Set {name} camera to MJPG@640x480 @ {fps}FPS')

    def publish_all(self, publishers, captures):
        def cap_pub(cap, pub, name):
            if not cap.isOpened():
                return
            if not cap.grab():
                self.get_logger().warning(f'{name} grab failed')
                return
            ret, frame = cap.retrieve()
            if not ret or frame is None:
                self.get_logger().warning(f'{name} retrieve failed')
                return
            try:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                pub.publish(msg)
            except CvBridgeError as e:
                self.get_logger().error(f'CvBridge error on {name}: {e}')

        # Publish only for cameras present in the configuration
        for name, cap in captures.items():
            if cap.isOpened():
                cap_pub(cap, publishers[name], name)

    def destroy_node(self):
        # Release all captures
        for cap in self.captures.values():
            if cap.isOpened():
                cap.release()
        super().destroy_node()

def load_config(config_file):
    with open(config_file, 'r') as file:
        return yaml.safe_load(file)

def main(args=None):
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="MultiCameraPublisher with configurable camera devices.")
    parser.add_argument('--config', type=str, help="Path to the camera configuration YAML file")
    args = parser.parse_args()

    # Load the configuration from YAML file
    config = load_config(args.config)

    # Properly initialize rclpy using sys.argv
    rclpy.init(args=sys.argv)  # Use sys.argv to pass the command-line arguments

    node = MultiCameraPublisher(config)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down MultiCameraPublisher...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
