# xarm7.py
import time
import threading
import numpy as np
import pinocchio as pin
from xarm import XArmAPI
from threading import Lock
import logging
from typing import Optional
from unibidex_client.motion_control.base import MotionControllerBase, ControlMode
from pinocchio import ReferenceFrame

logger = logging.getLogger(__name__)


def skew(v: np.ndarray) -> np.ndarray:
    """
    Return the skew-symmetric matrix of a 3-vector.
    """
    return np.array([
        [0.0,    -v[2],  v[1]],
        [v[2],    0.0,  -v[0]],
        [-v[1],  v[0],   0.0]
    ])


class PIDController:
    def __init__(self, kp: np.ndarray, ki: np.ndarray, kd: np.ndarray):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._prev_err = None
        self._cum_err = np.zeros_like(kp)

    def reset(self):
        self._prev_err = None
        self._cum_err = np.zeros_like(self.kp)
        logger.debug("PID controller reset.")

    def control(self, err: np.ndarray, dt: float) -> np.ndarray:
        if self._prev_err is None:
            self._prev_err = np.zeros_like(err)
        # integral
        self._cum_err += err * dt
        # derivative
        d_err = (err - self._prev_err) / dt
        # PID output
        output = self.kp * err + self.ki * self._cum_err + self.kd * d_err
        self._prev_err = err.copy()
        return output


class XArm7PinocchioController(MotionControllerBase):
    def __init__(
        self,
        urdf_path: str,
        ee_link_name: str = "link_eef",
        dt: float = 0.02,
        use_arm: bool = True,
        arm_ip: str = "192.168.1.201",
        use_servo_control: bool = False,
        safety_center: Optional[np.ndarray] = None,
        safety_radius: float = 0.3,
        inner_control_dt: float = 1.0/250,
        max_velocity: Optional[np.ndarray] = None,
        pid_gains: Optional[dict] = None,
        tcp_offset: pin.SE3 = pin.SE3.Identity()
    ):
        super().__init__()
        self.dt = dt
        self.inner_dt = inner_control_dt
        self.use_arm = use_arm
        self.use_servo = use_servo_control

        # Safety
        self.safety_center = safety_center.copy() if safety_center is not None else np.zeros(3)
        self.safety_radius = safety_radius
        logger.info(f"Safety center: {self.safety_center}, radius: {self.safety_radius} m")

        # Pinocchio model
        self.pin_model = pin.buildModelFromUrdf(str(urdf_path))
        self.pin_data = self.pin_model.createData()
        self.ee_frame_id = self.pin_model.getFrameId(ee_link_name)
        logger.info(f"URDF loaded: {urdf_path}, DOF = {self.pin_model.nq}")

        # Default velocity limits
        n = self.pin_model.nq
        self.max_velocity = max_velocity if max_velocity is not None else np.ones(n) * 0.8

        # Control gains
        self.K_joint = np.eye(n) * 50.0
        self.D_joint = np.eye(n) * 10.0
        self.K_cart = np.eye(6) * 50.0
        self.D_cart = np.eye(6) * 10.0

        # TCP offset
        self.tcp_offset = tcp_offset
        self.tcp_inv = self.tcp_offset.inverse()

        # IK parameters
        self.ik_damping = 1e-5 * np.eye(6)
        self.ik_eps = 1e-3

        # PID setup
        default_gains = {
            'kp': np.ones(n) * 6.0,
            'ki': np.zeros(n),
            'kd': np.ones(n) * 0.5
        }
        gains = pid_gains or default_gains
        self.pid = PIDController(gains['kp'], gains['ki'], gains['kd'])
        logger.info(f"PID gains: Kp={gains['kp']}, Ki={gains['ki']}, Kd={gains['kd']}")

        # Hardware loop setup
        self._arm_target: Optional[np.ndarray] = None
        self._arm_lock = Lock()
        self._stop_flag = False

        if self.use_arm:
            self.arm = XArmAPI(arm_ip, is_radian=True)
            self.reset()
            self._arm_thread = threading.Thread(
                target=self._internal_control_loop, daemon=True
            )
            self._arm_thread.start()
            logger.info(f"xArm interface started at {arm_ip}")

        # Current readings
        self._current_lock = Lock()
        self._currents = np.zeros(self.pin_model.nq, dtype=float)
        self._stop_current_thread = False
        self._current_thread = threading.Thread(
            target=self._current_poll_loop, daemon=True
        )
        self._current_thread.start()

        # —— Added: Start watchdog thread ——
        self._stop_watchdog = False
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop, daemon=True
        )
        # self._watchdog_thread.start()

    def get_dof(self) -> int:
        return self.pin_model.nq

    def set_safety_center(self, center: np.ndarray) -> None:
        self.safety_center = center.copy()
        logger.info(f"Safety center updated: {self.safety_center}")

    def set_safety_radius(self, radius: float) -> None:
        if radius < 0:
            raise ValueError("safety_radius must be non-negative")
        self.safety_radius = radius
        logger.info(f"Safety radius updated: {self.safety_radius} m")

    def reset(self):
        if not self.use_arm:
            return
        self.arm.motion_enable(True)
        mode = 1 if self.use_servo else 4
        self.arm.set_mode(mode)
        self.arm.set_state(0)
        time.sleep(0.5)
        self.pid.reset()
        logger.info("xArm reset and PID cleared.")

    def _current_poll_loop(self):
        """Continuously read joint currents in background and store in self._currents"""
        while not self._stop_current_thread:
            code, state = self.arm.get_joint_states(is_radian=True)
            if code == 0 and len(state) >= 3:
                cur = np.array(state[2], dtype=float)
                with self._current_lock:
                    self._currents[:] = cur
            else:
                logger.warning("Current reading failed, code=%s state=%s", code, state)
            time.sleep(self.inner_dt)

    # —— Added: Watchdog thread method ——
    def _watchdog_loop(self):
        """
        Periodically check if the end effector is within the safety sphere,
        and executes emergency stop if out of bounds.
        """
        check_interval = 0.05  # Check every 50ms
        while not self._stop_watchdog:
            try:
                tcp = self.get_current_tcp_pose()
                pos = tcp.translation
                dist = np.linalg.norm(pos - self.safety_center)
                if dist > self.safety_radius:
                    logger.error(
                        f"EE exceeds safety radius! Distance from center {dist:.3f} m (limit {self.safety_radius} m), executing emergency stop."
                    )
                    self.arm.motion_enable(False)  # XArmAPI 紧急停机
                    break
            except Exception as e:
                logger.warning(f"Watchdog thread error: {e}")
            time.sleep(check_interval)

    def stop(self):
        if not self.use_arm:
            return
        # Stop control and current monitoring threads
        self._stop_flag = True
        self._arm_thread.join()
        self._stop_current_thread = True
        self._current_thread.join()

        # —— Added: Stop and wait for watchdog thread to end ——
        self._stop_watchdog = True
        self._watchdog_thread.join()

        logger.info("xArm control & watchdog threads stopped.")

    def get_latest_currents(self) -> np.ndarray:
        with self._current_lock:
            return self._currents.copy()

    def _internal_control_loop(self):
        while not self._stop_flag:
            time.sleep(self.inner_dt)
            with self._arm_lock:
                target_q = self._arm_target
            if target_q is None:
                continue

            if not self._ee_in_safety(target_q):
                logger.warning(f"Target outside safety radius {self.safety_radius} m; rejected.")
                continue

            if self.use_servo:
                q_safe = self.clip_next_qpos(target_q)
                self.arm.set_servo_angle_j(q_safe.tolist(), speed=100, mvacc=1000, wait=False)
                logger.debug(f"Servo command to q: {q_safe}")
            else:
                curr_q = self.get_arm_qpos()
                err = target_q - curr_q
                qvel = self.pid.control(err, self.inner_dt)
                qvel = np.clip(qvel, -self.max_velocity, self.max_velocity)
                self.arm.vc_set_joint_velocity(qvel.tolist())
                logger.debug(f"Velocity command via PID: {qvel}")

    def get_arm_qpos(self) -> np.ndarray:
        _, state = self.arm.get_joint_states(is_radian=True)
        return np.array(state[0])

    def get_current_tcp_pose(self) -> pin.SE3:
        qpos = self.get_arm_qpos()
        return self.forward_kinematics(qpos) * self.tcp_offset

    def clip_next_qpos(self, target_qpos: np.ndarray) -> np.ndarray:
        curr = self.get_arm_qpos()
        err = target_qpos - curr
        scale = np.max(np.abs(err) / (self.max_velocity * self.inner_dt))
        return (curr + err/scale) if scale > 1 else target_qpos.copy()

    def forward_kinematics(self, qpos: np.ndarray) -> pin.SE3:
        pin.forwardKinematics(self.pin_model, self.pin_data, qpos)
        return pin.updateFramePlacement(self.pin_model, self.pin_data, self.ee_frame_id)

    def inverse_kinematics(
        self,
        ee_pose: pin.SE3,
        init_qpos: Optional[np.ndarray] = None,
        max_iter: int = 300,
    ) -> np.ndarray:
        q = init_qpos.copy() if init_qpos is not None else self.get_arm_qpos()
        target_ee = ee_pose
        for _ in range(max_iter):
            pin.forwardKinematics(self.pin_model, self.pin_data, q)
            ee_curr = pin.updateFramePlacement(self.pin_model, self.pin_data, self.ee_frame_id)
            ee_curr = ee_curr * self.tcp_offset
            err = pin.log(ee_curr.actInv(target_ee)).vector
            if np.linalg.norm(err) < self.ik_eps:
                break
            J = pin.computeFrameJacobian(self.pin_model, self.pin_data, q, self.ee_frame_id)
            dq = J.T.dot(np.linalg.solve(J.dot(J.T) + self.ik_damping, err))
            q = pin.integrate(self.pin_model, q, dq * self.dt)
        return q

    def _ee_in_safety(self, qpos: np.ndarray) -> bool:
        ee = self.forward_kinematics(qpos)
        pos = (ee * self.tcp_offset).translation
        inside = np.linalg.norm(pos - self.safety_center) <= self.safety_radius
        logger.debug(f"EE pos: {pos}, inside safety: {inside}")
        return inside

    def control_command(self, target_tcp: pin.SE3, init_qpos: Optional[np.ndarray] = None):
        rotation_degrees = self.calculate_rotation_around_world_axes(target_tcp)
        init_q = init_qpos if init_qpos is not None else self.get_arm_qpos()
        tgt_q = self.inverse_kinematics(target_tcp, init_q)
        with self._arm_lock:
            self._arm_target = tgt_q.copy()
        logger.info(f"Control target set to: {tgt_q}")

    def set_servo_angles(self, angles: np.ndarray):
        with self._arm_lock:
            self._arm_target = angles.copy()
        logger.info(f"Set servo angles: {angles}")

