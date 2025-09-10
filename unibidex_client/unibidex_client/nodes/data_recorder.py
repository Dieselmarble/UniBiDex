#!/usr/bin/env python3
import signal
import os
import time
import datetime
import cv2
import rclpy
import zarr
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from message_filters import Subscriber, ApproximateTimeSynchronizer


class DataRecorder(Node):
    """Use ApproximateTimeSynchronizer to sync seven topics and write to Zarr."""
    def __init__(self, zarr_path: str = None, slop: float = 0.05, queue_size: int = 500):
        super().__init__('data_recorder')
        self.bridge = CvBridge()
        self._idx = 0
        self._initialized = False
        self._start_time = time.time()

        # Zarr initialization
        if zarr_path is None:
            ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            zarr_path = os.path.expanduser(f'~/Desktop/teleop_data_{ts}.zarr')
        self.get_logger().info(f'📀 Initializing DataRecorder → {zarr_path}')
        store = zarr.DirectoryStore(zarr_path)
        self._root = zarr.group(store=store, overwrite=False)

        # ---- message_filters Subscribers ----
        sub_l_cmd = Subscriber(self, Float64MultiArray,   'xarm/left/joint_angle')
        sub_r_cmd = Subscriber(self, Float64MultiArray,   'xarm/right/joint_angle')
        sub_l_grip = Subscriber(self, Float64,             'xarm/left/gripper_opening')
        sub_r_grip = Subscriber(self, Float64,             'xarm/right/gripper_opening')
        sub_img_l  = Subscriber(self, Image,               'camera/left_wrist/image_raw')
        sub_img_r  = Subscriber(self, Image,               'camera/right_wrist/image_raw')
        sub_img_f  = Subscriber(self, Image,               'camera/front/image_raw')

        ats = ApproximateTimeSynchronizer(
            [sub_l_cmd, sub_r_cmd,
             sub_l_grip, sub_r_grip,
             sub_img_l,  sub_img_r,  sub_img_f],
            queue_size=queue_size,
            slop=slop,
            allow_headerless=True
        )
        ats.registerCallback(self._sync_callback)
        self.get_logger().info(f'ApproximateTimeSynchronizer ready (slop={slop}s, queue={queue_size}).')

    def _initialize_datasets(self, joint_dim: int):
        """Dynamically create Zarr datasets based on joint dimensions."""
        self._root.create_dataset('commands',
            shape=(0, 2, joint_dim), chunks=(1,2,joint_dim),
            dtype='f4', compressor=zarr.Blosc())
        self._root.create_dataset('gripper',
            shape=(0, 2), chunks=(1,2),
            dtype='f4', compressor=zarr.Blosc())
        self._root.create_dataset('images',
            shape=(0, 3, 480, 640, 3), chunks=(1,1,480,640,3),
            dtype='u1', compressor=zarr.Blosc())
        self._root.create_dataset('timestamps',
            shape=(0,), chunks=(1,), dtype='f8', compressor=zarr.Blosc())

        self._cmd_ds  = self._root['commands']
        self._grip_ds = self._root['gripper']
        self._img_ds  = self._root['images']
        self._ts_ds   = self._root['timestamps']
        self._initialized = True
        self.get_logger().info("Datasets initialized.")

    def _sync_callback(self,
                       l_cmd_msg: Float64MultiArray,
                       r_cmd_msg: Float64MultiArray,
                       l_grip_msg: Float64,
                       r_grip_msg: Float64,
                       img_l_msg: Image,
                       img_r_msg: Image,
                       img_f_msg: Image):
        """Called when a group of approximately synchronized messages arrive, write one frame."""
        # Initialize datasets (only on first time)
        if not self._initialized:
            dim = len(l_cmd_msg.data)
            self._initialize_datasets(dim)

        # ---- Write command data ----
        L = np.array(l_cmd_msg.data, dtype='f4')
        R = np.array(r_cmd_msg.data, dtype='f4')
        cmd_arr = np.stack([L, R], axis=0)
        self._cmd_ds.append(cmd_arr[np.newaxis, ...])

        # ---- Write gripper data ----
        grips = np.array([float(l_grip_msg.data), float(r_grip_msg.data)], dtype='f4')
        self._grip_ds.append(grips[np.newaxis, ...])

        # ---- Convert and write images ----
        try:
            f_l = self.bridge.imgmsg_to_cv2(img_l_msg, desired_encoding='bgr8')
            f_r = self.bridge.imgmsg_to_cv2(img_r_msg, desired_encoding='bgr8')
            f_f = self.bridge.imgmsg_to_cv2(img_f_msg, desired_encoding='bgr8')
        except (CvBridgeError, cv2.error) as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return
        # Resize images
        f_l = cv2.resize(f_l, (640, 480))
        f_r = cv2.resize(f_r, (640, 480))
        f_f = cv2.resize(f_f, (640, 480))
        img_stack = np.stack([f_l, f_r, f_f], axis=0)
        self._img_ds.append(img_stack[np.newaxis, ...])

        # ---- Write timestamps ----
        t = self.get_clock().now().nanoseconds * 1e-9
        self._ts_ds.append(np.array([t], dtype='f8'))

        self._idx += 1
        self.get_logger().info(f"Frame #{self._idx} recorded.")

    def destroy_node(self):
        elapsed = time.time() - self._start_time
        print(f'\r✔️ Finished: {self._idx} frames in {elapsed:.1f}s')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataRecorder()
    signal.signal(signal.SIGINT, lambda *_: rclpy.shutdown())
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
