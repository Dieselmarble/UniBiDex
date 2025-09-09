#!/usr/bin/env python3
import yaml
import time
import logging
import multiprocessing
import numpy as np
import rclpy
import sys
sys.path.append('/home/user/Documents/UniBiDex/unibidex/banana_teleoperation/unibidex_client')
from unibidex_client.motion_control.xarm7 import XArm7PinocchioController
from unibidex_client.motion_control.gripper_2f85 import Robotiq2F85Driver
import threading
from unibidex_client.nodes.unibidex_bridge_node import UniBiDexBridgeNode

# --- Logging setup ---
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger(__name__)


def load_config(config_path: str):
    with open(config_path, "r") as f:
        return yaml.safe_load(f)


class SingleUniBiDexController:
    """
    Sub-process level controller wrapper:
      - Initialize Pinocchio controller + Robotiq
      - Execute gripper commands asynchronously to avoid blocking
      - step_with_external_state only performs non-blocking gripper scheduling
    """
    def __init__(self, ctrl_cfg, side: str):

        self.logger = logging.getLogger(f"TeleopControllerProcess-{side}")
        self.logger.info("Initializing controller process.")

        self.side = side
        self.dt = ctrl_cfg["dt"]

        # gripper parameters and state
        gr_cfg = ctrl_cfg["gripper"]
        gripper_serial = gr_cfg["serial"]
        self.gripper_speed = gr_cfg.get("speed", 50.0)
        self.gripper_force = gr_cfg.get("force", 100.0)
        self.gripper_min_opening = gr_cfg.get("min_opening", 5.0)
        self.gripper_max_opening = gr_cfg.get("max_opening", 70.0)

        self.gripper = Robotiq2F85Driver(serial_number=gripper_serial)
        self.gripper.reset()
        time.sleep(0.5)

        # smooth opening target and thread control
        self.desired_opening = self.gripper.opening
        self._stop_gripper_thread = False
        self._gripper_thread = threading.Thread(target=self._gripper_loop, daemon=True)
        self._gripper_thread.start()

        # motion controller
        self.ctrl = XArm7PinocchioController(
            urdf_path=ctrl_cfg["urdf_path"],
            ee_link_name=ctrl_cfg.get("ee_link_name", "link_eef"),
            dt=self.dt,
            use_arm=True,
            arm_ip=ctrl_cfg["arm_ip"],
            use_servo_control=False,
            safety_radius=ctrl_cfg.get("safety_radius", 0.8),
            inner_control_dt=ctrl_cfg.get("inner_control_dt", 1.0/250),
        )
        self.ctrl.set_safety_center(self.ctrl.get_current_tcp_pose().translation)
        self.ctrl.set_safety_radius(0.5)
        # clear errors, power on
        self.ctrl.arm.clean_error()
        self.ctrl.arm.clean_warn()
        self.ctrl.arm.motion_enable(True)
        time.sleep(1)
        self.ctrl.arm.set_collision_sensitivity(0)
        time.sleep(1)
        self.ctrl.arm.set_state(state=0)

        self.logger.info(f"Controller for '{side}' initialized.")
    
    def _gripper_loop(self):
        """
        Background loop: read desired_opening, apply exponential smoothing, and send position commands at a fixed rate
        """
        prev = self.gripper.opening
        alpha = 0.2
        rate = 50  # Hz
        interval = 1.0 / rate
        while not self._stop_gripper_thread:
            target = np.clip(self.desired_opening,
                             self.gripper_min_opening,
                             self.gripper_max_opening)
            smooth = alpha * target + (1 - alpha) * prev
            prev = smooth
            try:
                self.gripper.go_to(
                    opening=smooth,
                    speed=self.gripper_speed,
                    force=self.gripper_force,
                    blocking_call=False
                )
            except Exception as e:
                self.logger.error(f"Gripper error: {e}")
            time.sleep(interval)

    def step_with_external_state(self, arm_cmd, gripper_cmd=0, teleop=True):
        """
        Main loop per step:
         - Update robot target
         - Write gripper target, no longer start a new thread
         - Sleep one control period
        """
        if not arm_cmd:
            time.sleep(self.dt)
            return

        # 1) parse command
        qj     = np.array(arm_cmd, dtype=float)
        gr_val = float(gripper_cmd)  # [0,1]

        # 2) update robot target
        with self.ctrl._arm_lock:
            self.ctrl._arm_target = qj.copy()
        self.logger.debug(f"[{self.side}] New arm target: {qj}")

        # 3) update gripper opening target
        # gr_val=0->fully open, 1->fully closed
        if teleop:
            opening = (1.0 - gr_val) * (self.gripper_max_opening - self.gripper_min_opening) + self.gripper_min_opening
        else:
            opening = gr_val
        self.desired_opening = opening

        # 4) sleep
        time.sleep(self.dt)

    def shutdown(self):
        # stop gripper thread and cleanup
        self._stop_gripper_thread = True
        self._gripper_thread.join(timeout=1.0)
        self.gripper.reset()
        # disable robot torque
        self.ctrl.arm.clean_error()
        self.ctrl.arm.motion_enable(False)


def run_controller_proc(ctrl_cfg, teleop_data, side):
    logger = logging.getLogger("ControllerProc")
    logger.info("Controller process started.")

    ctrl = SingleUniBiDexController(ctrl_cfg, side)

    def feedback_loop():
        rate_hz = ctrl_cfg.get("feedback_hz", int(1.0/ctrl.dt))
        interval = 1.0/rate_hz
        while True:
            # 1) read arm currents (shape: [7])
            arm_currents = ctrl.ctrl.get_latest_currents()
            # 2) read gripper current (scalar or shape: [1])
            gripper_current = 0.0 #ctrl.gripper.current
            # 3) combine into one array, e.g. last element store gripper current
            all_currents = np.concatenate([
                np.array(arm_currents, dtype=float),
                np.atleast_1d(gripper_current).astype(float)
            ])
            # 4) read joint angles
            joint_angles = ctrl.ctrl.get_arm_qpos()
            # 5) write back to shared dict
            teleop_data[f"{side}_currents"] = all_currents.tolist()
            teleop_data[f"{side}_angles"] = np.array(joint_angles, dtype=float).tolist()
            teleop_data[f"{side}_gripper_opening"] = float(ctrl.gripper.opening)
            time.sleep(interval)

    threading.Thread(target=feedback_loop, daemon=True).start()

    dt = ctrl_cfg["dt"]
    try:
        while True:
            key = f"{side}_cmd"
            cmd = teleop_data.get(key, None)
            if cmd:
                ctrl.step_with_external_state(cmd[:-1], cmd[-1])
            time.sleep(dt)
    except KeyboardInterrupt:
        logger.info("Controller process interrupted, exiting.")
    finally:
        ctrl.shutdown()
        logger.info("Controller process shutdown complete.")

def main():
    multiprocessing.set_start_method('spawn')
    mp_manager = multiprocessing.Manager()
    teleop_data = mp_manager.dict()

    config = load_config("/home/user/Documents/UniBiDex/unibidex/banana_teleoperation/unibidex_client/unibidex_client/configs/bimanual.yml")
    left_cfg = config["controllers"]["left"]
    right_cfg = config["controllers"]["right"]

    # ROS2 initialization and Bridge node
    rclpy.init()
    bridge_node = UniBiDexBridgeNode(teleop_data, feedback_hz=200.0)

    # Start left and right arm control processes
    l_proc = multiprocessing.Process(target=run_controller_proc, args=(left_cfg, teleop_data, 'left'), daemon=True)
    r_proc = multiprocessing.Process(target=run_controller_proc, args=(right_cfg, teleop_data, 'right'), daemon=True)
    l_proc.start()
    r_proc.start()

    try:
        logger.info("Spinning bridge_node …")
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt—shutting down.")
    finally:
        # Clean up
        # rec_proc.terminate()
        l_proc.terminate()
        r_proc.terminate()
        bridge_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
    