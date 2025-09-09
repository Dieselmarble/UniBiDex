#!/usr/bin/env python3
import yaml
import time
import logging
import multiprocessing
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from peel import SingleArmTeleopController
from std_msgs.msg import Float32, Float32MultiArray
import numpy as np
import pinocchio as pin

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger(__name__)

def load_config(config_path):
    with open(config_path, "r") as f:
        return yaml.safe_load(f)


class TeleopBridgeNode(Node):
    def __init__(self, teleop_data):
        super().__init__('teleop_bridge')
        self.teleop_data = teleop_data
        self.create_subscription(PoseStamped, 'retargeting/hand/left/ee_pose', self._left_pose_cb, 10)
        self.create_subscription(PoseStamped, 'retargeting/hand/right/ee_pose', self._right_pose_cb, 10)
        self.create_subscription(Bool, 'retargeting/gripper/left/command', self._left_gripper_cb, 10)
        self.create_subscription(Bool, 'retargeting/gripper/right/command', self._right_gripper_cb, 10)
        # 加入 joystick/rotate
        self.create_subscription(Float32, 'retargeting/rotate/left', self._left_rotate_cb, 10)
        self.create_subscription(Float32, 'retargeting/rotate/right', self._right_rotate_cb, 10)
        # 加入 joystick axis
        self.create_subscription(Float32MultiArray, 'retargeting/joystick/left', self._left_joystick_cb, 10)
        self.create_subscription(Float32MultiArray, 'retargeting/joystick/right', self._right_joystick_cb, 10)

    def _left_pose_cb(self, msg):
        self.teleop_data["left_pose"] = msg

    def _right_pose_cb(self, msg):
        self.teleop_data["right_pose"] = msg

    def _left_gripper_cb(self, msg):
        self.teleop_data["left_gripper"] = msg.data

    def _right_gripper_cb(self, msg):
        self.teleop_data["right_gripper"] = msg.data

    def _left_rotate_cb(self, msg):
        self.teleop_data["left_rotate"] = msg.data

    def _right_rotate_cb(self, msg):
        self.teleop_data["right_rotate"] = msg.data

    def _left_joystick_cb(self, msg):
        # msg.data = [x, y]
        self.teleop_data["left_joystick"] = msg.data if msg.data else [0.0, 0.0]

    def _right_joystick_cb(self, msg):
        self.teleop_data["right_joystick"] = msg.data if msg.data else [0.0, 0.0]


def run_controller_proc(controller_config, teleop_data, side):

    def make_se3(pose_msg):
        p = np.array([pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z])
        q = np.array([pose_msg.pose.orientation.w, pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z])
        return pin.SE3(pin.Quaternion(q), p)

    controller = SingleArmTeleopController(
        urdf_path=controller_config["urdf_path"],
        ee_link=controller_config["ee_link"],
        arm_ip=controller_config["arm_ip"],
        gripper_serial=controller_config["gripper"]["serial"],
        gripper_speed=controller_config["gripper"]["speed"],
        gripper_force=controller_config["gripper"]["force"],
        use_real_arm=controller_config["use_real_arm"],
        dt=controller_config["dt"],
        translation_scale=controller_config["translation_scale"],
        rotation_scale=controller_config["rotation_scale"],
        controller_side=side,
        teleop_node=None,
        tcp_offset=controller_config["tcp_offset"],
    )

    logger = logging.getLogger(f"TeleopControllerProc-{side}")
    logger.info("Controller process started.")
    try:
        while True:
           if side == "left":
               pose_msg   = teleop_data.get("left_pose",   None)
               gripper_cmd= teleop_data.get("left_gripper",None)
               axes       = teleop_data.get("left_joystick",[0.0,0.0])
           else:
               pose_msg   = teleop_data.get("right_pose",   None)
               gripper_cmd= teleop_data.get("right_gripper",None)
               axes       = teleop_data.get("right_joystick",[0.0,0.0])
           if pose_msg is not None:
                curr_ctrl_se3 = make_se3(pose_msg)
                # 假设controller支持step_with_external_state(curr_ctrl_se3, rotate_q7, gripper_cmd, joystick)
                controller.step_with_external_state(
                    curr_ctrl_se3,
                    axes,
                    gripper_cmd,
                )
                time.sleep(controller.dt)
    except KeyboardInterrupt:
        logger.info("Shutting down controller.")


def main():
    multiprocessing.set_start_method('spawn')
    mp_manager = multiprocessing.Manager()
    teleop_data = mp_manager.dict()
    config = load_config("/home/user/Documents/UniBiDex/unibidex/banana_teleoperation/unibidex_client/unibidex_client/configs/bimanual.yml")
    left_config = config["controllers"]["left"]
    right_config = config["controllers"]["right"]

    rclpy.init()
    bridge_node = TeleopBridgeNode(teleop_data)
    logger.info("TeleopBridgeNode initialized.")

    # 启动左右臂进程
    p_left = multiprocessing.Process(target=run_controller_proc, args=(left_config, teleop_data, "left"), daemon=True)
    p_right = multiprocessing.Process(target=run_controller_proc, args=(right_config, teleop_data, "right"), daemon=True)
    p_left.start()
    p_right.start()

    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt—shutting down all.")
    finally:
        p_left.terminate()
        p_right.terminate()
        bridge_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
