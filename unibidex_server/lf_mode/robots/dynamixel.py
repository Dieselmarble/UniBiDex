from typing import Dict, Optional, Sequence, Tuple
import numpy as np
from lf_mode.robots.robot import Robot


class DynamixelRobot(Robot):
    """A class representing a UR robot."""

    def __init__(
        self,
        joint_ids: Sequence[int],
        joint_offsets: Optional[Sequence[float]] = None,
        joint_signs: Optional[Sequence[int]] = None,
        real: bool = True,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 4000000,
        gripper_config: Optional[Tuple[int, float, float]] = None,
        start_joints: Optional[np.ndarray] = None,
    ):
        from gello.dynamixel.driver import (
            DynamixelDriver,
        )

        print(f"attempting to connect to port: {port}")
        self.gripper_open_close: Optional[Tuple[float, float]]
        # Append gripper ID and offset/sign if present
        if gripper_config is not None:
            assert joint_offsets is not None and joint_signs is not None
            joint_ids = tuple(joint_ids) + (gripper_config[0],)
            joint_offsets = tuple(joint_offsets) + (0.0,)
            joint_signs = tuple(joint_signs) + (1,)
            self.gripper_open_close = (
                gripper_config[1] * np.pi / 180,
                gripper_config[2] * np.pi / 180,
            )
        else:
            self.gripper_open_close = None

        self._joint_ids = tuple(joint_ids)
        self._driver: DynamixelDriver

        # Store offsets and signs
        self._joint_offsets = np.array(joint_offsets) if joint_offsets is not None else np.zeros(len(self._joint_ids))
        self._joint_signs = np.array(joint_signs) if joint_signs is not None else np.ones(len(self._joint_ids))

        assert len(self._joint_ids) == len(self._joint_offsets) == len(self._joint_signs)

        # Initialize driver
        if real:
            self._driver = DynamixelDriver(self._joint_ids, port=port, baudrate=baudrate)
        else:
            pass
        self._torque_on = False
        self._last_pos = None
        self._alpha = 0.98

        # Calibrate start joints if provided
        if start_joints is not None:
            # align offsets to start_joints
            offsets = []
            current = self.get_joint_state()[:len(start_joints)]
            for idx, (c, s, off) in enumerate(zip(current, start_joints, self._joint_offsets)):
                delta = c - s
                wrap = 2 * np.pi * np.round(delta / (2 * np.pi))
                offsets.append(off + wrap * self._joint_signs[idx])
            if self.gripper_open_close is not None:
                offsets.append(self._joint_offsets[-1])
            self._joint_offsets = np.array(offsets)

    def num_dofs(self) -> int:
        return len(self._joint_ids)

    def get_joint_state(self) -> np.ndarray:
        raw = self._driver.get_joints()
        pos = (raw - self._joint_offsets) * self._joint_signs
        assert len(pos) == self.num_dofs()
        # map gripper to [0,1]
        if self.gripper_open_close is not None:
            lo, hi = self.gripper_open_close
            g = np.clip((pos[-1] - lo) / (hi - lo), 0, 1)
            pos[-1] = g
        # smooth
        if self._last_pos is None:
            self._last_pos = pos
        else:
            pos = self._last_pos * (1 - self._alpha) + pos * self._alpha
            self._last_pos = pos
        return pos


    # def get_joint_state(self) -> np.ndarray:
    #     pos = (self._driver.get_joints() - self._joint_offsets) * self._joint_signs
    #     assert len(pos) == self.num_dofs()

    #     if self.gripper_open_close is not None:
    #         # map pos to [0, 1]
    #         g_pos = (pos[-1] - self.gripper_open_close[0]) / (
    #             self.gripper_open_close[1] - self.gripper_open_close[0]
    #         )
    #         g_pos = min(max(0, g_pos), 1)
    #         pos[-1] = g_pos

    #     if self._last_pos is None:
    #         self._last_pos = pos
    #     else:
    #         # exponential smoothing
    #         pos = self._last_pos * (1 - self._alpha) + pos * self._alpha
    #         self._last_pos = pos

    #     return pos

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        # send only first N-1 joints via position sync write
        self._driver.set_joints((joint_state + self._joint_offsets).tolist())

    def set_torque_mode(self, mode: bool):
        if mode == self._torque_on:
            return
        self._driver.set_torque_mode(mode)
        self._torque_on = mode

    def set_current(self, currents: np.ndarray):
        # send only arm torques, exclude gripper if any
        # print(f"Setting currents: {currents}")
        self._driver.set_current(currents.tolist())

    def get_observations(self) -> Dict[str, np.ndarray]:
        return {"joint_state": self.get_joint_state()}