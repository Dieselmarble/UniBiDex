#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float32, Float32MultiArray
from threading import Lock
from typing import Optional, Tuple

class TeleopClientNode(Node):
    def __init__(self):
        super().__init__('teleop_client')

        # Latest target poses (left, right)
        self.left_pose: Optional[PoseStamped] = None
        self.right_pose: Optional[PoseStamped] = None

        # Latest gripper commands (True=open, False=close)
        self.left_gripper_cmd: Optional[bool] = None
        self.right_gripper_cmd: Optional[bool] = None

        # Latest joystick axis values (float)
        self.left_axis: Optional[float] = None
        self.right_axis: Optional[float] = None

        self.left_rotate: Optional[float] = None
        self.right_rotate: Optional[float] = None

        self.lock = Lock()

        # Subscribe to end-effector poses
        self.create_subscription(
            PoseStamped,
            'retargeting/hand/left/ee_pose',
            self._left_pose_callback,
            10
        )
        self.create_subscription(
            PoseStamped,
            'retargeting/hand/right/ee_pose',
            self._right_pose_callback,
            10
        )

        # Subscribe to gripper commands
        self.create_subscription(
            Bool,
            'retargeting/gripper/left/command',
            self._left_gripper_callback,
            10
        )
        self.create_subscription(
            Bool,
            'retargeting/gripper/right/command',
            self._right_gripper_callback,
            10
        )

        # Subscribe to joystick axes (Float32MultiArray [x,y])
        self.create_subscription(
            Float32MultiArray,
            'retargeting/joystick/left',
            self._left_axis_callback,
            10
        )
        self.create_subscription(
            Float32MultiArray,
            'retargeting/joystick/right',
            self._right_axis_callback,
            10
        )

        self.create_subscription(
            Float32,
            'retargeting/rotate/left',
            self._left_rotate_z_callback,
            10
        )
        self.create_subscription(
            Float32,
            'retargeting/rotate/right',
            self._right_rotate_z_callback,
            10
        )

        self.get_logger().info(
            'TeleopClientNode started: subscribed to ee_pose, gripper and joystick topics.'
        )

    # Pose callbacks
    def _left_pose_callback(self, msg: PoseStamped):
        with self.lock:
            self.left_pose = msg

    def _right_pose_callback(self, msg: PoseStamped):
        with self.lock:
            self.right_pose = msg

    # Gripper callbacks
    def _left_gripper_callback(self, msg: Bool):
        with self.lock:
            self.left_gripper_cmd = msg.data

    def _right_gripper_callback(self, msg: Bool):
        with self.lock:
            self.right_gripper_cmd = msg.data

    # Joystick axis callbacks (only first axis)
    def _left_axis_callback(self, msg: Float32MultiArray):
        with self.lock:
            # Only use the first axis value
            self.left_axis = msg.data[0] if msg.data else None

    def _right_axis_callback(self, msg: Float32MultiArray):
        with self.lock:
            self.right_axis = msg.data[0] if msg.data else None

    def _left_rotate_z_callback(self, msg: Float32):
        with self.lock:
            # Only use the first axis value for rotation around Z
            self.left_rotate = msg.data if msg.data else None
    
    def _right_rotate_z_callback(self, msg: Float32):
        with self.lock:
            # Only use the first axis value for rotation around Z
            self.right_rotate = msg.data if msg.data else None

    # Public getters
    def get_latest_poses(self) -> Tuple[Optional[PoseStamped], Optional[PoseStamped]]:
        with self.lock:
            return self.left_pose, self.right_pose

    def get_left_gripper_command(self) -> Optional[bool]:
        with self.lock:
            return self.left_gripper_cmd

    def get_right_gripper_command(self) -> Optional[bool]:
        with self.lock:
            return self.right_gripper_cmd

    def get_left_axis(self) -> Optional[float]:
        with self.lock:
            return self.left_axis

    def get_right_axis(self) -> Optional[float]:
        with self.lock:
            return self.right_axis
        
    def get_left_rotate_z(self) -> Optional[float]:
        with self.lock:
            return self.left_rotate
    
    def get_right_rotate_z(self) -> Optional[float]:
        with self.lock:
            return self.right_rotate

# Global node reference
teleop_client_node: Optional[TeleopClientNode] = None


def init_teleop_client_node() -> TeleopClientNode:
    rclpy.init()
    global teleop_client_node
    teleop_client_node = TeleopClientNode()
    return teleop_client_node


def spin_teleop_client_node():
    rclpy.spin(teleop_client_node)
