#!/usr/bin/env python3
import logging

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class UniBiDexBridgeNode(Node):
    def __init__(self, teleop_data, feedback_hz: float = 200.0):
        super().__init__('unibidex_bridge')
        self.teleop_data = teleop_data
        self.bridge = CvBridge()

        # 1) 关节命令
        self.create_subscription(Float64MultiArray, 'unibidex/left/joint_command',  self._cmd_left,  10)
        self.create_subscription(Float64MultiArray, 'unibidex/right/joint_command', self._cmd_right, 10)

        # 3) 电流与角度发布
        self.left_current_pub  = self.create_publisher(Float64MultiArray, 'xarm/left/joint_current',  10)
        self.right_current_pub = self.create_publisher(Float64MultiArray, 'xarm/right/joint_current', 10)
        self.left_angle_pub    = self.create_publisher(Float64MultiArray, 'xarm/left/joint_angle',    10)
        self.right_angle_pub   = self.create_publisher(Float64MultiArray, 'xarm/right/joint_angle',   10)
        self.left_gripper_pub  = self.create_publisher(Float64, 'xarm/left/gripper_opening', 10)
        self.right_gripper_pub = self.create_publisher(Float64, 'xarm/right/gripper_opening', 10)
        self.create_timer(1.0/feedback_hz, self._publish_feedback)

        # 打开 Debug 级别，查看每帧 encoding
        self.get_logger().info("UniBiDexBridgeNode initialized.")

    # ——— Command callbacks ———
    def _cmd_left(self,  msg):  self.teleop_data['left_cmd']  = list(msg.data)
    def _cmd_right(self, msg):  self.teleop_data['right_cmd'] = list(msg.data)


    # ——— Publish currents ———
    def _publish_feedback(self):
        left = self.teleop_data.get('left_currents')
        if left is not None:
            self.left_current_pub.publish(Float64MultiArray(data=left))
        right = self.teleop_data.get('right_currents')
        if right is not None:
            self.right_current_pub.publish(Float64MultiArray(data=right))

        left_angle = self.teleop_data.get('left_angles')
        if left_angle is not None:
            self.left_angle_pub.publish(Float64MultiArray(data=left_angle))
        right_angle = self.teleop_data.get('right_angles')
        if right_angle is not None:
            self.right_angle_pub.publish(Float64MultiArray(data=right_angle))

        left_opening = self.teleop_data.get('left_gripper_opening')
        if left_opening is not None:
            self.left_gripper_pub.publish(Float64(data=float(left_opening)))
        right_opening = self.teleop_data.get('right_gripper_opening')
        if right_opening is not None:
            self.right_gripper_pub.publish(Float64(data=float(right_opening)))

    def destroy_node(self):
        super().destroy_node()
