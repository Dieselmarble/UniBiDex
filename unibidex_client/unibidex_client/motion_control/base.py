# base.py
from abc import ABC, abstractmethod
from enum import Enum, auto
import numpy as np
import pinocchio as pin
import time
import logging

logger = logging.getLogger(__name__)

class ControlMode(Enum):
    POSITION = auto()
    VELOCITY = auto()
    TORQUE = auto()
    IMPEDANCE = auto()

class MotionControllerBase(ABC):
    def __init__(self):
        self._mode = ControlMode.POSITION
        self._target = None
        logger.info("Initialized MotionControllerBase with default POSITION control mode.")

    def set_control_mode(self, mode: ControlMode) -> None:
        if not isinstance(mode, ControlMode):
            raise ValueError(f"mode must be a ControlMode, got {mode!r}")
        self._mode = mode
        logger.info(f"Control mode set to: {mode.name}")

    def set_target(self, target):
        if hasattr(target, 'copy'):
            self._target = target.copy()
        else:
            self._target = target
        logger.debug("Target set for control.")

    def compute_control(self, qpos: np.ndarray, qvel: np.ndarray, dt: float) -> np.ndarray:
        logger.debug(f"Computing control with mode: {self._mode.name}")
        if self._mode == ControlMode.POSITION:
            return self._position_control(qpos, dt)
        elif self._mode == ControlMode.VELOCITY:
            return self._velocity_control(qvel, dt)
        elif self._mode == ControlMode.TORQUE:
            return self._torque_control(qpos, qvel, dt)
        elif self._mode == ControlMode.IMPEDANCE:
            return self._impedance_control(qpos, qvel, dt)
        else:
            raise RuntimeError(f"Unhandled control mode: {self._mode}")

    @abstractmethod
    def get_dof(self) -> int: ...
    
    @abstractmethod
    def forward_kinematics(self, qpos: np.ndarray) -> pin.SE3: ...

    @abstractmethod
    def inverse_kinematics(self, target_pose: pin.SE3, init_qpos: np.ndarray = None) -> np.ndarray: ...

    def integrate_velocity(self, qpos: np.ndarray, qvel: np.ndarray, dt: float) -> np.ndarray:
        return qpos + qvel * dt
