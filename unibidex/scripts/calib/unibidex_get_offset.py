#!/usr/bin/env python3
"""
Refactored unibidex_get_offset script: load all parameters from a single YAML config file.
"""
import logging
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

import numpy as np
import yaml

from unibidex_core.dynamixel.driver import DynamixelDriver

# Constants for offset search
OFFSET_RANGE = 8 * np.pi
OFFSET_STEPS = int(OFFSET_RANGE / (np.pi / 2)) * 2 + 1  # +/-8π in π/2 steps

@dataclass(frozen=True)
class Config:
    config_file: Path

    port: str
    baudrate: int
    start_joints: List[float]
    joint_signs: List[int]
    joint_ids: Optional[List[int]] = None
    gripper: bool = True

    @classmethod
    def load(cls, path: Path) -> "Config":
        if not path.exists():
            raise FileNotFoundError(f"Config file not found: {path}")
        data = yaml.safe_load(path.read_text())
        return cls(
            config_file=path,
            port=str(data['port']),
            baudrate=int(data['baudrate']),
            start_joints=list(map(float, data['start_joints'])),
            joint_signs=list(map(int, data['joint_signs'])),
            joint_ids=list(map(int, data['joint_ids'])) if data.get('joint_ids') else None,
            gripper=bool(data.get('gripper', True)),
        )

    def __post_init__(self):
        # Validate lengths
        if len(self.start_joints) != len(self.joint_signs):
            raise ValueError("start_joints and joint_signs must match length")
        # Validate signs
        for idx, s in enumerate(self.joint_signs):
            if s not in (-1, 1):
                raise ValueError(f"Invalid joint_sign at {idx}: {s}")
        # Default joint_ids
        expected = len(self.start_joints) + (1 if self.gripper else 0)
        if self.joint_ids is None:
            object.__setattr__(self, 'joint_ids', list(range(1, expected + 1)))
        elif len(self.joint_ids) != expected:
            raise ValueError(f"joint_ids length {len(self.joint_ids)} != expected {expected}")


def validate_port(port: Path) -> None:
    if not port.exists():
        raise FileNotFoundError(f"Serial port not found: {port}")
    if not port.is_char_device():
        raise ValueError(f"Port is not a character device: {port}")


def compute_best_offsets(
    joint_states: np.ndarray,
    start_angles: List[float],
    signs: List[int],
) -> List[float]:
    offsets = []
    candidates = np.linspace(-OFFSET_RANGE, OFFSET_RANGE, OFFSET_STEPS)
    for i, (state, target, sign) in enumerate(zip(joint_states, start_angles, signs)):
        best_offset, min_err = 0.0, float('inf')
        for c in candidates:
            err = abs(sign * (state - c) - target)
            if err < min_err:
                min_err, best_offset = err, c
        offsets.append(best_offset)
        logging.debug(f"Joint {i}: offset={best_offset:.4f}, error={min_err:.4f}")
    return offsets


def get_joint_states(driver: DynamixelDriver, retries: int = 5) -> np.ndarray:
    for i in range(retries):
        try:
            return driver.get_joints()
        except Exception as e:
            logging.warning(f"Read attempt {i+1} failed: {e}")
    raise RuntimeError("Failed to read joints after retries")


def main():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} CONFIG_YAML")
        sys.exit(1)

    config_path = Path(sys.argv[1])
    cfg = Config.load(config_path)

    validate_port(Path(cfg.port))
    logging.info(f"Loaded config from: {cfg.config_file}")
    logging.info(f"Port: {cfg.port}, Baudrate: {cfg.baudrate}")
    logging.info(f"Joint IDs: {cfg.joint_ids}")

    try:
        driver = DynamixelDriver(ids=cfg.joint_ids, port=cfg.port, baudrate=cfg.baudrate)
    except Exception as e:
        logging.exception("Failed initializing DynamixelDriver")
        sys.exit(1)

    # Warm-up
    for _ in range(3): driver.get_joints()

    joints = get_joint_states(driver)
    logging.info(f"Current joint states: {np.round(joints,3).tolist()}")

    offsets = compute_best_offsets(joints[:len(cfg.start_joints)], cfg.start_joints, cfg.joint_signs)
    vals = [f"{o:.3f}" for o in offsets]
    mults = [f"{round(o/(np.pi/2))}*π/2" for o in offsets]
    print("\nBest Offsets:")
    print("  Values:", vals)
    print("  Multiples of π/2:", mults)

    if cfg.gripper:
        angle = joints[-1]
        print(f"Gripper open (deg): {np.degrees(angle)-0.2:.1f}")
        print(f"Gripper close (deg): {np.degrees(angle)-42:.1f}")

    driver.close()


if __name__ == "__main__":
    try:
        main()
    except Exception:
        logging.exception("Script failed")
        sys.exit(1)
