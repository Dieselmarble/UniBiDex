import os
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

from unibidex_core.agents.agent import Agent
from unibidex_core.robots.dynamixel import DynamixelRobot


@dataclass
class DynamixelRobotConfig:
    joint_ids: Sequence[int]
    """The joint ids of UniBiDex (not including the gripper). Usually (1, 2, 3 ...)."""

    joint_offsets: Sequence[float]
    """The joint offsets of UniBiDex. There needs to be a joint offset for each joint_id and should be a multiple of pi/2."""

    joint_signs: Sequence[int]
    """The joint signs of UniBiDex. There needs to be a joint sign for each joint_id and should be either 1 or -1.

    This will be different for each arm design. Refernce the examples below for the correct signs for your robot.
    """

    gripper_config: Tuple[int, int, int]
    """The gripper config of UniBiDex. This is a tuple of (gripper_joint_id, degrees in open_position, degrees in closed_position)."""

    def __post_init__(self):
        assert len(self.joint_ids) == len(self.joint_offsets)
        assert len(self.joint_ids) == len(self.joint_signs)

    def make_robot(
        self, port: str = "/dev/ttyUSB0", start_joints: Optional[np.ndarray] = None
    ) -> DynamixelRobot:
        return DynamixelRobot(
            joint_ids=self.joint_ids,
            joint_offsets=list(self.joint_offsets),
            real=True,
            joint_signs=list(self.joint_signs),
            port=port,
            baudrate=4000000,  # Use a higher baudrate for faster communication
            gripper_config=self.gripper_config,
            start_joints=start_joints,
        )


PORT_CONFIG_MAP: Dict[str, DynamixelRobotConfig] = {
    # xArm left
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAJZ5RY-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6, 7),
        joint_offsets=(
            2 * np.pi / 2,
            2 * np.pi / 2,
            2 * np.pi / 2,
            1.5 * np.pi / 2,
            1.5 * np.pi / 2,
            1 * np.pi / 2,
            6.3116 * np.pi / 2,
        ),
        joint_signs=(1, 1, 1, 1, 1, 1, 1),
        gripper_config=(8, 195, 152),
    ),
    # xArm right
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAUZ35W-if00-port0": DynamixelRobotConfig(
        joint_ids=(9, 10, 11, 12, 13, 14, 15),
        joint_offsets=[
            2 * np.pi / 2,
            2 * np.pi / 2,
            2 * np.pi / 2,
            1.5 * np.pi / 2,
            2.5 * np.pi / 2,
            1 * np.pi / 2,
            1.3116 * np.pi / 2,
        ],
        joint_signs=(1, 1, 1, 1, 1, 1, 1),
        gripper_config=(16, 195, 152),
    ),

}

class UniBiDexAgent(Agent):
    def _init_current_buffer(self):
        self._last_currents = None
        self._current_alpha = 0.2  # Smoothing coefficient, adjustable
    def __init__(
        self,
        port: str,
        dynamixel_config: Optional[DynamixelRobotConfig] = None,
        start_joints: Optional[np.ndarray] = None,
    ):
        # Initialize robot
        if dynamixel_config is not None:
            self._robot = dynamixel_config.make_robot(
                port=port, start_joints=start_joints
            )
        else:
            assert os.path.exists(port), port
            assert port in PORT_CONFIG_MAP, f"Port {port} not in config map"
            self._robot = PORT_CONFIG_MAP[port].make_robot(port=port)

        # Switch to Current-based Position Control Mode (Mode 5)
        ADDR_OPERATING_MODE = 11
        MODE_CURR_POS_CTRL = 5
        # Temporarily disable torque to write EEPROM
        self._robot.set_torque_mode(False)
        for dxl_id in self._robot._driver._ids:
            res, err = self._robot._driver._packetHandler.write1ByteTxRx(
                self._robot._driver._portHandler,
                dxl_id,
                ADDR_OPERATING_MODE,
                MODE_CURR_POS_CTRL,
            )

        # Flags for dither in friction compensation
        self.stiction_dither_flag = np.ones(self._robot.num_dofs(), dtype=bool)
        self._init_current_buffer()

    def xarm_current_to_xl330_cmd(
        self,
        I_xarm: np.ndarray,
        Ktx: float = 0.02,
        Kt_xl: float = 0.11,
        lsb: float = 0.00269,
        max_val: int = 1023,
        deadband: int = 30,
        torque_scale: float = 0.05,
        invert: bool = False,
    ) -> np.ndarray:
        if I_xarm is None or np.allclose(I_xarm, 0):
            return np.zeros_like(I_xarm, dtype=float)
        tau = Ktx * I_xarm
        I_cmd = tau / Kt_xl
        raw = np.round(I_cmd / lsb).astype(float)
        mag = np.abs(raw)
        mag = np.where(mag < deadband, 0, mag)
        mag = np.clip(mag, 0, max_val)
        val = np.sign(raw) * mag
        val = np.round(val * torque_scale).astype(float)
        return -val if invert else val

    def friction_compensation(self, vel: np.ndarray) -> np.ndarray:
        tau_g = np.zeros(self._robot.num_dofs())
        tau_ss = np.zeros_like(tau_g)
        gain = 1.0
        vel_thresh = 0.1
        for i in range(len(vel)):
            if abs(vel[i]) < vel_thresh:
                comp = gain * abs(tau_g[i])
                tau_ss[i] = comp if self.stiction_dither_flag[i] else -comp
                self.stiction_dither_flag[i] = not self.stiction_dither_flag[i]
        return tau_ss

    def act(self, obs: Dict[str, np.ndarray]) -> np.ndarray:
        # Re-enable torque after mode change
        self._robot.set_torque_mode(True)
        joint_pos, joint_vel = self._robot._driver.get_positions_and_velocities()
        currents_in = obs.get('currents', np.zeros(self._robot.num_dofs()))

        # --- Smooth Transition ---
        if self._last_currents is None:
            smoothed_currents = currents_in.copy()
        else:
            smoothed_currents = self._current_alpha * currents_in + (1 - self._current_alpha) * self._last_currents
        self._last_currents = smoothed_currents.copy()

        torque_cmd = self.xarm_current_to_xl330_cmd(smoothed_currents)
        # Amplify commands for the last two joints
        # if torque_cmd.size >= 2:
        #     torque_cmd[-2:] = torque_cmd[-2:] * 2.0
        torque_cmd += self.friction_compensation(joint_vel)

        # Current-based Position Control: lock pos, limit torque
        # self._robot._driver.set_joints(joint_pos)
        self._robot._driver.set_current(torque_cmd.astype(int))

        return self._robot.get_joint_state()
