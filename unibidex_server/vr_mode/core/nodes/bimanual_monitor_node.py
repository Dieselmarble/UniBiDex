from copy import deepcopy
from threading import Lock
from typing import Tuple
from enum import Enum

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool, Float32, Float32MultiArray
from geometry_msgs.msg import PoseStamped, Pose
from hand_controller_msgs.msg import BimanualHandDetection, BimanualControllerState
from pytransform3d import rotations
from utils.robot_utils import project_average_rotation
from utils.init_config import BimanualAlignmentMode
from tqdm import tqdm
from utils.robot_utils import QUESTTRANSLATION2ROBOT, QUESTROTATION2ROBOT
from utils.robot_utils import LPFilter, LPRotationFilter


def ros_pose_to_pos_quat(msg: Pose) -> Tuple[np.ndarray, np.ndarray]:
    p = msg.position
    q = msg.orientation
    return np.array([p.x, p.y, p.z]), np.array([q.w, q.x, q.y, q.z])


class ControlMode(Enum):
    HAND = "hand"
    CONTROLLER = "ctrl"


class TeleopMonitorNode(Node):
    def __init__(
        self,
        hand_topic: str,
        ctrl_topic: str,
        need_init: bool = True,
        verbose: bool = False,
        control_mode: ControlMode = ControlMode.HAND
    ):
        super().__init__("teleop_monitor")
        self.verbose = verbose
        self.control_mode = control_mode
        self.get_logger().info(f"Initializing TeleopMonitorNode in {self.control_mode.name} mode...")

        self.basis_map_translation = QUESTTRANSLATION2ROBOT
        self.basis_map_rotation = QUESTROTATION2ROBOT

        self.calib_n_samples = 20
        self._calib_pending = None
        self._calib_buffer = []
        self._calib_tqdm = None

        self.prev_left_stick_pressed = False
        self.prev_publish_ok_pressed = False
        self.publishing_enabled = False
        self.last_left_x = False
        self.last_left_y = False
        self.last_right_a = False
        self.last_right_b = False


        self.left_stick_pub = self.create_publisher(Float32MultiArray, "retargeting/joystick/left", 50)
        self.right_stick_pub = self.create_publisher(Float32MultiArray, "retargeting/joystick/right", 50)

        self._cb_group = MutuallyExclusiveCallbackGroup()

        self.hand_sub = self.create_subscription(
            BimanualHandDetection,
            f"{hand_topic}/results",
            self.on_hand_detection,
            qos_profile=10,
            callback_group=self._cb_group
        )
        self.get_logger().info(f"Subscribed to hand topic: {hand_topic}/results")

        self.ctrl_sub = self.create_subscription(
            BimanualControllerState,
            ctrl_topic,
            self.on_ctrl_detection,
            qos_profile=10,
            callback_group=self._cb_group
        )
        self.get_logger().info(f"Subscribed to controller topic: {ctrl_topic}")

        self.left_pose_pub = self.create_publisher(PoseStamped, "retargeting/hand/left/ee_pose", 50)
        self.right_pose_pub = self.create_publisher(PoseStamped, "retargeting/hand/right/ee_pose", 50)
        self.left_gripper_pub = self.create_publisher(Bool, "retargeting/gripper/left/command", 50)
        self.right_gripper_pub = self.create_publisher(Bool, "retargeting/gripper/right/command", 50)
        self.left_rotate_pub = self.create_publisher(Float32, "retargeting/rotate/left", 50)
        self.right_rotate_pub = self.create_publisher(Float32, "retargeting/rotate/right", 50)

        self.hand_initialized = not need_init
        self.ctrl_initialized = not need_init
        self.calibrated_hand_pos = (np.zeros(3), np.zeros(3))
        self.calibrated_hand_rot = (np.eye(3), np.eye(3))
        self.calibrated_ctrl_pos = (np.zeros(3), np.zeros(3))
        self.calibrated_ctrl_rot = (np.eye(3), np.eye(3))

        self.hand_pose_lock = Lock()
        self.ctrl_pose_lock = Lock()
        self.last_hand_wrist_pose = [np.zeros(7), np.zeros(7)]
        self.last_ctrl_pose = [np.zeros(7), np.zeros(7)]
        self.ema_hand_pose = [np.zeros(7), np.zeros(7)]
        self.ema_ctrl_pose = [np.zeros(7), np.zeros(7)]
        self.hand_jump_thresh = 1.5
        self.ctrl_jump_thresh = 1.5
        self.rot_jump_thresh_deg = 30.0
        self.ema_alpha = 0.85

        self.lp_filters = [
            [LPFilter(self.ema_alpha), LPFilter(self.ema_alpha)],
            [LPFilter(self.ema_alpha), LPFilter(self.ema_alpha)]
        ]
        self.rot_filters = [
            [LPRotationFilter(self.ema_alpha), LPRotationFilter(self.ema_alpha)],
            [LPRotationFilter(self.ema_alpha), LPRotationFilter(self.ema_alpha)]
        ]

        self.bimanual_alignment_mode = BimanualAlignmentMode.ALIGN_SEPARATELY
        self.get_logger().info("TeleopMonitorNode initialized.")


    def on_hand_detection(self, data: BimanualHandDetection):
        if self.control_mode != ControlMode.HAND:
            return
        if not data.detected or len(data.left_joints) != 63:
            self.get_logger().warn("Invalid or incomplete hand data, skipping.")
            return

        left_pos, left_quat = ros_pose_to_pos_quat(data.left_wrist_pose)
        right_pos, right_quat = ros_pose_to_pos_quat(data.right_wrist_pose)
        left_joints = np.reshape(data.left_joints, [21, 3])
        right_joints = np.reshape(data.right_joints, [21, 3])

        # Initial calibration via sampling
        if not self.hand_initialized:
            if self._hands_open(left_joints, right_joints) and self._calib_pending is None:
                self.get_logger().info(f"Starting HAND calibration sampling ({self.calib_n_samples} frames)…")
                self._calib_pending = 'hand'
                self._calib_buffer = []
                self._calib_tqdm = tqdm(total=self.calib_n_samples, desc="Hand calibration", unit="frame")

            if self._calib_pending == 'hand':
                self._calib_buffer.append((left_pos, left_quat, right_pos, right_quat))
                if self._calib_tqdm:
                    self._calib_tqdm.update(1)
                if len(self._calib_buffer) >= self.calib_n_samples:
                    # compute averages
                    lpos_arr = np.stack([s[0] for s in self._calib_buffer])
                    rpos_arr = np.stack([s[2] for s in self._calib_buffer])
                    lquat_arr = np.stack([s[1] for s in self._calib_buffer])
                    rquat_arr = np.stack([s[3] for s in self._calib_buffer])

                    # centralized calibration
                    self._perform_calibration(
                        lpos_arr, lquat_arr,
                        rpos_arr, rquat_arr,
                        device_type="hand"
                    )

                    # cleanup sampling
                    if self._calib_tqdm:
                        self._calib_tqdm.close()
                        self._calib_tqdm = None
                    self._calib_pending = None
                return

        # Normal processing
        self._process_detection(
            left_pos, left_quat,
            right_pos, right_quat,
            device_type="hand",
            lock=self.hand_pose_lock,
            last_pose=self.last_hand_wrist_pose,
            ema_pose=self.ema_hand_pose,
            jump_thresh=self.hand_jump_thresh,
            calibrated_pos=self.calibrated_hand_pos,
            calibrated_rot=self.calibrated_hand_rot,
        )


    def on_ctrl_detection(self, data: BimanualControllerState):
        if self.control_mode != ControlMode.CONTROLLER:
            return

        # —— 1. 校准触发：任何时候按下左摇杆，都可以重新 init calibration —— #
        left_pressed = data.left_stick_pressed
        if left_pressed and not self.prev_left_stick_pressed:
            if self._calib_tqdm:
                self._calib_tqdm.close()
            self.get_logger().info(
                f"Starting CONTROLLER calibration sampling ({self.calib_n_samples} frames)…"
            )
            self._calib_pending = 'ctrl'
            self._calib_buffer = []
            self._calib_tqdm = tqdm(
                total=self.calib_n_samples,
                desc="Controller calibration",
                unit="frame"
            )
        self.prev_left_stick_pressed = left_pressed

        # 如果正处于采样状态，就一直往 buffer 里放，无视发布状态：
        if self._calib_pending == 'ctrl':
            left_pos, left_quat = ros_pose_to_pos_quat(data.left_pose)
            right_pos, right_quat = ros_pose_to_pos_quat(data.right_pose)
            self._calib_buffer.append((left_pos, left_quat, right_pos, right_quat))
            if self._calib_tqdm:
                self._calib_tqdm.update(1)
            if len(self._calib_buffer) >= self.calib_n_samples:
                # 计算平均并执行集中式校准
                lpos_arr = np.stack([s[0] for s in self._calib_buffer])
                rpos_arr = np.stack([s[2] for s in self._calib_buffer])
                lquat_arr = np.stack([s[1] for s in self._calib_buffer])
                rquat_arr = np.stack([s[3] for s in self._calib_buffer])

                avg_lpos = lpos_arr.mean(axis=0)
                avg_rpos = rpos_arr.mean(axis=0)
                avg_lquat = project_average_rotation(lquat_arr)
                avg_rquat = project_average_rotation(rquat_arr)

                self._perform_calibration(
                    avg_lpos, avg_lquat,
                    avg_rpos, avg_rquat,
                    device_type="ctrl"
                )

                # 采样完成，清理
                if self._calib_tqdm:
                    self._calib_tqdm.close()
                    self._calib_tqdm = None
                self._calib_pending = None
            # 采样阶段不往下继续执行
            return

        # —— 2. A 键（right_buttons[0]）切换发布状态 —— #
        publish_ok_pressed = bool(data.right_buttons and data.right_buttons[0] and data.left_buttons and data.left_buttons[2])
        if self.prev_publish_ok_pressed and not publish_ok_pressed:
            self.publishing_enabled = not self.publishing_enabled
            state = "resumed" if self.publishing_enabled else "paused"
            self.get_logger().info(f"Publishing end effector poses {state}.")
        self.prev_publish_ok_pressed = publish_ok_pressed

        # —— 3. 如果没有打开发布，就不做后续的夹爪和正常处理 —— #
        if not self.publishing_enabled:
            return

        # —— 4. 夹爪控制逻辑 —— #
        # Front trigger closes; Side trigger opens
        if data.left_index_trigger > 0.5:
            self.left_gripper_pub.publish(Bool(data=False))
        elif data.left_hand_trigger > 0.5:
            self.left_gripper_pub.publish(Bool(data=True))

        if data.right_index_trigger > 0.5:
            self.right_gripper_pub.publish(Bool(data=False))
        elif data.right_hand_trigger > 0.5:
            self.right_gripper_pub.publish(Bool(data=True))

        # --- New: publish joystick axes ---
        left_axes = Float32MultiArray(data=[data.left_stick[0], data.left_stick[1]])
        right_axes = Float32MultiArray(data=[data.right_stick[0], data.right_stick[1]])
        self.left_stick_pub.publish(left_axes)
        self.right_stick_pub.publish(right_axes)


        left_x_pressed = data.left_buttons[2]   # X
        left_y_pressed = data.left_buttons[3]   # Y
        right_a_pressed = data.right_buttons[0] # A
        right_b_pressed = data.right_buttons[1] # B
        
        ROTATE_SPEED = 1.0
        left_rotate = 0.0
        if left_x_pressed:
            left_rotate = -ROTATE_SPEED
        elif left_y_pressed:
            left_rotate = ROTATE_SPEED

        right_rotate = 0.0
        if right_a_pressed:
            right_rotate = -ROTATE_SPEED
        elif right_b_pressed:
            right_rotate = ROTATE_SPEED
        
        self.left_rotate_pub.publish(Float32(data=left_rotate))
        self.right_rotate_pub.publish(Float32(data=right_rotate))

        # —— 5. 正常位姿处理 —— #
        left_pos, left_quat = ros_pose_to_pos_quat(data.left_pose)
        right_pos, right_quat = ros_pose_to_pos_quat(data.right_pose)
        
        self._process_detection(
            left_pos, left_quat,
            right_pos, right_quat,
            device_type="ctrl",
            lock=self.ctrl_pose_lock,
            last_pose=self.last_ctrl_pose,
            ema_pose=self.ema_ctrl_pose,
            jump_thresh=self.ctrl_jump_thresh,
            calibrated_pos=self.calibrated_ctrl_pos,
            calibrated_rot=self.calibrated_ctrl_rot,
        )

    def _process_detection(
        self,
        left_pos: np.ndarray,
        left_quat: np.ndarray,
        right_pos: np.ndarray,
        right_quat: np.ndarray,
        device_type: str,
        lock: Lock,
        last_pose: list,
        ema_pose: list,
        jump_thresh: float,
        calibrated_pos: Tuple[np.ndarray, np.ndarray],
        calibrated_rot: Tuple[np.ndarray, np.ndarray],
    ):
        for idx, (pos, quat) in enumerate([(left_pos, left_quat), (right_pos, right_quat)]):
            p0 = calibrated_pos[idx]
            R0 = calibrated_rot[idx]
            Rm = rotations.matrix_from_quaternion(quat)
            Rl = R0.T @ Rm
            pl = R0.T @ (pos - p0)
            ql = rotations.quaternion_from_matrix(Rl)

            with lock:
                last = last_pose[idx]
            trans_ok = np.linalg.norm(last[:3] - pl) < jump_thresh or np.allclose(last[:3], 0)
            rot_ok = np.linalg.norm(last[3:] - ql) < np.deg2rad(self.rot_jump_thresh_deg) or np.allclose(last[3:], 0)
            if trans_ok and rot_ok:
                device_idx = 0 if device_type == "hand" else 1
                filt_pos = self.lp_filters[device_idx][idx].next(pl)
                filt_quat = self.rot_filters[device_idx][idx].next(ql / np.linalg.norm(ql))
                filt_pose = np.concatenate([filt_pos, filt_quat])

                with lock:
                    ema_pose[idx] = filt_pose
                    last_pose[idx] = filt_pose
                self._publish(idx, filt_pose)

    def _publish(self, idx: int, pose_np: np.ndarray):
        if not self.publishing_enabled:
            return

        # Position remap
        quest_pos = pose_np[:3]
        arm_pos = self.basis_map_translation @ quest_pos

        # Orientation remap with quaternion renormalization
        q = pose_np[3:]
        norm = np.linalg.norm(q)
        if norm < 1e-8:
            self.get_logger().warn("Quaternion too small, skipping orientation.")
            return
        q = q / norm

        quest_R = rotations.matrix_from_quaternion(q)
        arm_R = self.basis_map_rotation @ quest_R @ self.basis_map_rotation.T
        arm_quat = rotations.quaternion_from_matrix(arm_R)

        pub = self.left_pose_pub if idx == 0 else self.right_pose_pub
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = arm_pos
        msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z = arm_quat
        pub.publish(msg)

    def _compute_hand_joint_angles(self, joints: np.ndarray) -> np.ndarray:
        tip_idx = np.array([4, 8, 12, 16, 20])
        palm_idx = np.array([1, 5, 9, 13, 17])
        root = joints[0:1, :]
        tips = joints[tip_idx]
        palm = joints[palm_idx]
        tip_vec = tips - root
        tip_vec /= np.linalg.norm(tip_vec, axis=1, keepdims=True)
        palm_vec = palm - root
        palm_vec /= np.linalg.norm(palm_vec, axis=1, keepdims=True)
        angle = np.arccos(np.clip(np.sum(tip_vec * palm_vec, axis=1), -1.0, 1.0))
        return angle

    def _hands_open(self, left: np.ndarray, right: np.ndarray) -> bool:
        angles_l = self._compute_hand_joint_angles(left)
        angles_r = self._compute_hand_joint_angles(right)
        lo, hi = 0.01, np.deg2rad(15)
        return (lo < angles_l.mean() < hi) and (lo < angles_r.mean() < hi)

    def _perform_calibration(
        self,
        left_pos: np.ndarray,
        left_quat: np.ndarray,
        right_pos: np.ndarray,
        right_quat: np.ndarray,
        device_type: str,
    ):
        mode = self.bimanual_alignment_mode

        if mode == BimanualAlignmentMode.ALIGN_CENTER:
            avg_q = project_average_rotation(np.array([right_quat]))
            avg_p = (left_pos + right_pos) * 0.5
            R = rotations.matrix_from_quaternion(avg_q)
            init_pos = (avg_p, avg_p)
            init_rot = (R, R)
        elif mode == BimanualAlignmentMode.ALIGN_LEFT:
            avg_q = project_average_rotation(np.array([left_quat]))
            avg_p = left_pos
            R = rotations.matrix_from_quaternion(avg_q)
            init_pos = (avg_p, avg_p)
            init_rot = (R, R)
        elif mode == BimanualAlignmentMode.ALIGN_RIGHT:
            avg_q = project_average_rotation(np.array([right_quat]))
            avg_p = right_pos
            R = rotations.matrix_from_quaternion(avg_q)
            init_pos = (avg_p, avg_p)
            init_rot = (R, R)
        elif mode == BimanualAlignmentMode.ALIGN_SEPARATELY:
            avg_ql = project_average_rotation(np.array([left_quat]))
            avg_pl = left_pos
            Rl = rotations.matrix_from_quaternion(avg_ql)
            avg_qr = project_average_rotation(np.array([right_quat]))
            avg_pr = right_pos
            Rr = rotations.matrix_from_quaternion(avg_qr)
            init_pos = (avg_pl, avg_pr)
            init_rot = (Rl, Rr)
        else:
            raise NotImplementedError(f"Unknown mode {mode}")

        if device_type == "hand":
            self.calibrated_hand_pos = init_pos
            self.calibrated_hand_rot = init_rot
            self.hand_initialized = True
            self.get_logger().info(f"Hand calibration complete in {mode.name} mode")
        else:
            self.calibrated_ctrl_pos = init_pos
            self.calibrated_ctrl_rot = init_rot
            self.ctrl_initialized = True
            self.get_logger().info(f"Controller calibration complete in {mode.name} mode")


def main():
    rclpy.init()
    node = TeleopMonitorNode(
        hand_topic="/quest3/bimanual_hand_detection",
        ctrl_topic="/quest3/bimanual_controller_state",
        need_init=True,
        verbose=False,
        control_mode=ControlMode.CONTROLLER
    )
    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down TeleopMonitorNode (KeyboardInterrupt).")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()