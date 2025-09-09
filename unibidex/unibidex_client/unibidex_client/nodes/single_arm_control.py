import time
import threading
import logging
from typing import Optional
import sys
sys.path.append('/home/user/Documents/UniBiDex/unibidex/banana_teleoperation/unibidex_client/unibidex_client')
import numpy as np
import pinocchio as pin
from pinocchio.utils import zero
from unibidex_client.motion_control.gripper_2f85 import Robotiq2F85Driver
from unibidex_client.motion_control.xarm7 import XArm7PinocchioController
from unibidex_client.motion_control.base import ControlMode
from unibidex_client.motion_control.teleop_client_node import (
    init_teleop_client_node,
    spin_teleop_client_node,
)

# ——— Logging setup ———
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger(__name__)


def make_se3(pose_msg) -> pin.SE3:
    """Convert a ROS‐style pose message to a Pinocchio SE3."""
    p = np.array([
        pose_msg.pose.position.x,
        pose_msg.pose.position.y,
        pose_msg.pose.position.z,
    ])
    q = np.array([
        pose_msg.pose.orientation.w,
        pose_msg.pose.orientation.x,
        pose_msg.pose.orientation.y,
        pose_msg.pose.orientation.z,
    ])
    return pin.SE3(pin.Quaternion(q), p)


class TerminalRotationVisualizer:
    def __init__(self):
        self.last_update = time.time()
        self.update_interval = 0.1  # Update every 100ms
    
    def add_angles(self, roll, pitch, yaw):
        current_time = time.time()
        if current_time - self.last_update < self.update_interval:
            return
            
        self.last_update = current_time
        
        # Print rotation angles in a formatted way
        print("\033c", end="")  # Clear screen
        print("=== Rotation Angles (degrees) ===")
        print(f"Roll (X):  {roll:+7.2f}° {'#' * min(abs(int(roll/10)), 10)}")
        print(f"Pitch (Y): {pitch:+7.2f}° {'#' * min(abs(int(pitch/10)), 10)}")
        print(f"Yaw (Z):   {yaw:+7.2f}° {'#' * min(abs(int(yaw/10)), 10)}")
        print("================================")

# Create global visualizer instance
angle_visualizer = TerminalRotationVisualizer()

def visualize_rotation_angles(rotation_matrix):
    """Convert rotation matrix to Euler angles and display them in terminal."""
    # Convert rotation matrix to Euler angles
    euler_angles = pin.rpy.matrixToRpy(rotation_matrix)
    
    # Convert to degrees
    roll_deg = np.degrees(euler_angles[0])
    pitch_deg = np.degrees(euler_angles[1])
    yaw_deg = np.degrees(euler_angles[2])
    
    # Log the values
    logger.info(f"Rotation angles (degrees): Roll (x): {roll_deg:.2f}, "
                f"Pitch (y): {pitch_deg:.2f}, Yaw (z): {yaw_deg:.2f}")
    
    # Update the visualizer
    angle_visualizer.add_angles(roll_deg, pitch_deg, yaw_deg)


class SingleArmTeleopController:
    def __init__(
        self,
        urdf_path: str,
        ee_link: str,
        arm_ip: str,
        gripper_serial: str,
        gripper_speed: float = 50.0,
        gripper_force: float = 50.0,
        use_real_arm: bool = True,
        dt: float = 0.05,
        translation_scale: float = 0.5,
        rotation_scale: float = 1.0,
        controller_side: str = "left",
        teleop_node=None,
        tcp_offset: Optional[np.array] = None,
    ):
        # Teleop params
        self.dt = dt
        self.translation_scale = translation_scale
        self.rotation_scale = rotation_scale
        self.controller_side = controller_side
        self.teleop_node = teleop_node

        # ---- Adaptive scaling params ----
        self.min_translation_scale = 0.2    # 精细操作时的放大
        self.max_translation_scale = 1.5    # 粗调放大
        self.ref_translation_speed = 0.06   # m/s, 超过此速度则放大到最大

        self.min_rotation_scale = 0.4       # 精细旋转放大
        self.max_rotation_scale = 1.2       # 粗调旋转放大
        self.ref_rotation_speed = 0.10      # rad/s, 超过此速度则旋转放大到最大

        self._last_ctrl_p = None            # 存储上一帧手柄位置
        self._last_ctrl_R = None            # 存储上一帧手柄姿态

        # TCP offset if provided
        if tcp_offset is not None:
            rotation_matrix = np.asarray(tcp_offset['rotation'])
            translation_vector = np.asarray(tcp_offset['translation'])
            self.tcp_offset = pin.SE3(rotation_matrix, translation_vector)
        else:
            self.tcp_offset = pin.SE3(np.eye(3), np.zeros(3))

        # XArm controller
        self.ctrl = XArm7PinocchioController(
            urdf_path=urdf_path,
            ee_link_name=ee_link,
            dt=dt,
            use_arm=use_real_arm,
            arm_ip=arm_ip,
            use_servo_control=False,
            safety_radius=0.8,
            inner_control_dt=1.0/250,
            tcp_offset=self.tcp_offset,
        )
        # Initialize safety center and control mode
        init_center = self.ctrl.get_current_tcp_pose().translation
        self.ctrl.set_safety_center(init_center)
        self.ctrl.set_control_mode(ControlMode.VELOCITY)

        # Gripper driver
        self.gripper = Robotiq2F85Driver(serial_number=gripper_serial)
        self.gripper.reset()
        self.gripper_speed = gripper_speed
        self.gripper_force = gripper_force

        # Arm pose initialization
        self.robot_init_se3: pin.SE3 = None
        self.R_robot_init: np.ndarray = None
        self.R_ctrl_init: np.ndarray = None

        logger.info(f"Single‐arm+gripper teleop initialized on {controller_side} side.")

    def open_gripper(self, opening: float = 85.0):
        """Open gripper to specified opening in mm."""
        self.gripper.go_to(opening=opening,
                          speed=self.gripper_speed,
                          force=self.gripper_force)

    def close_gripper(self, opening: float = 0.0):
        """Close gripper (to specified opening in mm)."""
        self.gripper.go_to(opening=opening,
                          speed=self.gripper_speed,
                          force=self.gripper_force)
    
    def rotate_tcp_around_world_axis(self, axis: str, angle_deg: float, ini_t):
        """
        让末端 TCP 绕 世界坐标系 的 X/Y/Z 轴 原地旋转 angle_deg 度（保持当前位置不变）。
        
        :param axis: 'x'、'y' 或 'z'
        :param angle_deg: 旋转角度，正值按右手规则
        """
        # 1) 当前 world → TCP SE3
        current_se3 = self.ctrl.get_current_tcp_pose()
        R_curr      = current_se3.rotation
        p_curr      = current_se3.translation

        # 2) 选轴并构造 world‐axis 的增量旋转
        axes = {
            'x': np.array([1.0, 0.0, 0.0]),
            'y': np.array([0.0, 1.0, 0.0]),
            'z': np.array([0.0, 0.0, 1.0]),
        }
        if axis not in axes:
            raise ValueError("axis 必须是 'x'、'y' 或 'z'")
        axis_vec = axes[axis]
        angle   = np.deg2rad(angle_deg)
        R_delta = pin.exp3(axis_vec * angle)

        # 3) 旋转朝向，位置保持不变
        R_target = R_delta @ R_curr
        target_se3 = pin.SE3(R_target, ini_t)

        # 4) 调用 control_command（内部会用你那个 LOCAL_WORLD_ALIGNED 的 IK）
        self.ctrl.control_command(target_se3)
        
    def run(self):
        logger.info(f"Running {self.controller_side} controller...")
        try:
            while True:
                # Fetch pose from teleop node
                if self.controller_side == "left":
                    pose_msg, _ = self.teleop_node.get_latest_poses()
                    gripper_cmd = self.teleop_node.get_left_gripper_command()
                else:
                    _, pose_msg = self.teleop_node.get_latest_poses()
                    gripper_cmd = self.teleop_node.get_right_gripper_command()

                if pose_msg is None:
                    time.sleep(self.dt)
                    continue

                # Convert to SE3
                curr_ctrl_se3 = make_se3(pose_msg)
                p_ctrl = curr_ctrl_se3.translation
                R_ctrl = curr_ctrl_se3.rotation

                # On first frame, record initial poses
                if self.robot_init_se3 is None:
                    self.robot_init_se3 = self.ctrl.get_current_tcp_pose()
                    self.R_robot_init = self.robot_init_se3.rotation
                    self.R_ctrl_init = R_ctrl.copy()
                    # 存储上一次手柄位置/旋转
                    self._last_ctrl_p = p_ctrl.copy()
                    self._last_ctrl_R = R_ctrl.copy()
                    logger.info("Recorded initial robot and controller poses.")
                    time.sleep(self.dt)
                    continue

                # ------------- 1. Adaptive translation scaling -------------
                delta_p_hand = p_ctrl - self._last_ctrl_p
                v_hand = np.linalg.norm(delta_p_hand) / self.dt
                t_scale = self.min_translation_scale + (self.max_translation_scale - self.min_translation_scale) * min(1.0, v_hand / self.ref_translation_speed)
                # 2. 计算相对初始点的平移，并加缩放
                delta_p = p_ctrl * t_scale
                target_p = self.robot_init_se3.translation + delta_p

                # ------------- 2. Adaptive rotation scaling -------------
                delta_R = self._last_ctrl_R.T @ R_ctrl
                rotvec = pin.log3(delta_R)
                w_hand = np.linalg.norm(rotvec) / self.dt
                r_scale = self.min_rotation_scale + (self.max_rotation_scale - self.min_rotation_scale) * min(1.0, w_hand / self.ref_rotation_speed)

                # 旋转缩放
                R_rel = self.R_ctrl_init.T @ R_ctrl
                rot_vec = pin.log3(R_rel)
                R_rel_scaled = pin.exp3(rot_vec * r_scale)
                R_target_relative_baselink = self.ctrl.project_rotation_to_baselink(R_rel_scaled)
                R_target = self.R_robot_init @ R_target_relative_baselink     # 相对baselink

                # Command arm
                target_se3 = pin.SE3(R_target, target_p)
                visualize_rotation_angles(R_target)
                self.ctrl.control_command(target_se3)

                # 存储本周期的手柄位置/姿态，用于下一轮速度计算
                self._last_ctrl_p = p_ctrl.copy()
                self._last_ctrl_R = R_ctrl.copy()

                if gripper_cmd is not None:
                    if not gripper_cmd:
                        logger.info(f"Closing {self.controller_side} gripper.")
                        self.close_gripper()
                    else:
                        logger.info(f"Opening {self.controller_side} gripper.")
                        self.open_gripper()

                time.sleep(self.dt)

        except KeyboardInterrupt:
            logger.info("Shutting down arm+gripper.")
            if self.ctrl.use_arm:
                self.ctrl.stop()


if __name__ == "__main__":
    teleop_node = init_teleop_client_node()
    threading.Thread(target=spin_teleop_client_node, daemon=True).start()

    controller = SingleArmTeleopController(
        urdf_path="/home/user/Documents/UniBiDex/unibidex/banana_teleoperation/assets/urdf/arms/xarm7/xarm7.urdf",
        ee_link="link_eef",
        arm_ip="192.168.100.203",
        gripper_serial="DAA5H91L",
        gripper_speed=60.0,
        gripper_force=30.0,
        use_real_arm=True,
        dt=0.05,
        translation_scale=1.0,
        rotation_scale=1.0,
        controller_side="right",
        teleop_node=teleop_node,
        tcp_offset=dict(
            rotation=[[1, 0, 0], [0, 1, 0], [0, 0, 1]],
            translation=[0.0, 0.0, 0.174]
        )
    )
    controller.run()
