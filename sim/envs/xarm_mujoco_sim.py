#!/usr/bin/env python3
# filepath: banana_learning/banana_learning/ours/envs/xarm_mujoco_sim.py

# Set environment variables for better display compatibility
import os
# If you encounter GLX errors, try these settings:
# os.environ['MUJOCO_GL'] = 'egl'  # or 'osmesa' for software rendering
# os.environ['PYOPENGL_PLATFORM'] = 'egl'

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import threading
import time
import sys
from pathlib import Path

# Add environment path
env_path = Path(__file__).parent
sys.path.insert(0, str(env_path))

from bimanual_env import BimanualXArm7Env
from pathlib import Path

# Add environment path
env_path = Path(__file__).parent.parent / "banana_learning/banana_learning/ours/envs"
sys.path.insert(0, str(env_path))

from bimanual_env import BimanualXArm7Env

class XArmMujocoSimNode(Node):
    """
    XArm dual-arm MuJoCo simulation node based on BimanualXArm7Env
    Subscribes to: /gello/left/joint_command, /gello/right/joint_command
    Publishes: /xarm/left/joint_angle, /xarm/left/joint_current, /xarm/left/gripper_opening
              /xarm/right/joint_angle, /xarm/right/joint_current, /xarm/right/gripper_opening
    """
    
    def __init__(self):
        super().__init__('xarm_mujoco_sim_node')
        
        # Initialize MuJoCo environment
        self.init_env()
        
        # Create subscribers
        self.left_cmd_sub = self.create_subscription(
            Float64MultiArray,
            '/gello/left/joint_command',
            self.left_command_callback,
            10
        )
        
        self.right_cmd_sub = self.create_subscription(
            Float64MultiArray,
            '/gello/right/joint_command', 
            self.right_command_callback,
            10
        )
        
        # Create publishers
        self.left_joint_pub = self.create_publisher(Float64MultiArray, '/xarm/left/joint_angle', 10)
        self.left_current_pub = self.create_publisher(Float64MultiArray, '/xarm/left/joint_current', 10)
        self.left_gripper_pub = self.create_publisher(Float64MultiArray, '/xarm/left/gripper_opening', 10)
        
        self.right_joint_pub = self.create_publisher(Float64MultiArray, '/xarm/right/joint_angle', 10)
        self.right_current_pub = self.create_publisher(Float64MultiArray, '/xarm/right/joint_current', 10)
        self.right_gripper_pub = self.create_publisher(Float64MultiArray, '/xarm/right/gripper_opening', 10)
        
        # Control command cache
        self.left_command = np.zeros(8)  # 7 joints + 1 gripper
        self.right_command = np.zeros(8)
        self.command_lock = threading.Lock()
        
        # Start simulation thread
        self.sim_thread = threading.Thread(target=self.simulation_loop, daemon=True)
        self.sim_running = True
        self.sim_thread.start()
        
        # State publishing timer (50Hz)
        self.state_timer = self.create_timer(0.02, self.publish_states)
        
        self.get_logger().info("XArm MuJoCo simulation node started")
    
    def init_env(self):
        """Initialize MuJoCo environment"""
        try:
            # Create BimanualXArm7 environment (with visualization enabled)
            self.env = BimanualXArm7Env(
                use_grippers=True,
                render_mode='human'  # Enable visualization
            )
            
            # Reset environment
            obs, info = self.env.reset()
            
            self.get_logger().info(f"MuJoCo environment initialized successfully")
            self.get_logger().info(f"State dimension: {self.env.state_dim}")
            self.get_logger().info(f"Action dimension: {self.env.action_dim}")
            self.get_logger().info(f"Model qpos dimension: {self.env.model.nq}")
            self.get_logger().info(f"Model actuator dimension: {self.env.model.nu}")
            
        except Exception as e:
            self.get_logger().error(f"Environment initialization failed: {e}")
            # If visualization fails, try headless mode
            try:
                self.get_logger().warn("Trying headless mode...")
                self.env = BimanualXArm7Env(
                    use_grippers=True,
                    render_mode=None  # Headless mode
                )
                obs, info = self.env.reset()
                self.get_logger().info("Headless mode initialization successful")
            except Exception as e2:
                self.get_logger().error(f"Headless mode also failed: {e2}")
                raise
    
    def left_command_callback(self, msg):
        """Left arm command callback"""
        with self.command_lock:
            if len(msg.data) >= 7:
                # Ensure array length is 8, if only 7 joints, keep current gripper value
                self.left_command[:len(msg.data)] = msg.data[:8]
                if len(msg.data) == 7:
                    # If only 7 joints, set gripper to middle position
                    self.left_command[7] = 127.5  # Gripper middle position (0-255 range)
                elif len(msg.data) == 8:
                    # If 8 values (including gripper), map gripper value from 0-1 range to 0-255 range
                    # Note: leader arm gr_val=0 means fully open, gr_val=1 means fully closed
                    # MuJoCo 0=fully closed, 255=fully open
                    gripper_val = msg.data[7]  # 0-1 range from leader arm
                    # Correct mapping: leader's 1(closed) -> MuJoCo's 255(closed), leader's 0(open) -> MuJoCo's 0(open)
                    self.left_command[7] = gripper_val * 255.0
        

    
    def right_command_callback(self, msg):
        """Right arm command callback"""
        with self.command_lock:
            if len(msg.data) >= 7:
                # Ensure array length is 8, if only 7 joints, keep current gripper value
                self.right_command[:len(msg.data)] = msg.data[:8]
                if len(msg.data) == 7:
                    # If only 7 joints, set gripper to middle position
                    self.right_command[7] = 127.5  # Gripper middle position
                elif len(msg.data) == 8:
                    # If 8 values (including gripper), map gripper value from 0-1 range to 0-255 range
                    # Note: leader arm gr_val=0 means fully open, gr_val=1 means fully closed
                    # MuJoCo 0=fully closed, 255=fully open
                    gripper_val = msg.data[7]  # 0-1 range from leader arm
                    # Correct mapping: leader's 1(closed) -> MuJoCo's 255(closed), leader's 0(open) -> MuJoCo's 0(open)
                    self.right_command[7] = gripper_val * 255.0
        

    
    def simulation_loop(self):
        """Main simulation loop (100Hz)"""
        dt = 0.01  # 10ms
        
        while self.sim_running:
            start_time = time.time()
            
            # Build complete action vector
            with self.command_lock:
                # Combine left and right arm commands [left_7joints, left_gripper, right_7joints, right_gripper]
                action = np.concatenate([
                    self.left_command,   # Left arm 7 joints + gripper
                    self.right_command   # Right arm 7 joints + gripper
                ])
            
            # Execute environment step
            try:
                obs, reward, done, truncated, info = self.env.step(action)
                
                # Render display
                self.env.render()
                
            except Exception as e:
                self.get_logger().warn(f"Simulation step error: {e}")
            
            # Control simulation frequency
            elapsed = time.time() - start_time
            sleep_time = max(0, dt - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    def get_joint_positions(self):
        """Get joint positions"""
        try:
            # Get joint positions from environment
            if self.env.use_grippers:
                # Left arm 7 joints
                left_joints = self.env.data.qpos[:8]
                # Right arm 7 joints  
                right_joints = self.env.data.qpos[9:16]
            else:
                left_joints = self.env.data.qpos[:7]
                right_joints = self.env.data.qpos[7:14]
            
            return left_joints, right_joints
        except Exception as e:
            self.get_logger().warn(f"Failed to get joint positions: {e}")
            return np.zeros(7), np.zeros(7)
    
    def get_gripper_positions(self):
        """Get gripper positions"""
        try:
            # Default gripper positions
            left_gripper_pos = 0.0
            right_gripper_pos = 0.0
            
            # Get gripper positions from environment
            if self.env.use_grippers:
                n_qpos = self.env.model.nq
                if n_qpos > 7:
                    left_gripper_pos = self.env.data.qpos[7]  # Left gripper
                if n_qpos > 15:
                    right_gripper_pos = self.env.data.qpos[15]  # Right gripper
                
                # Convert gripper position from MuJoCo's 0-255 range back to 0-1 range for publishing
                # MuJoCo 0=fully closed, 255=fully open
                # Correct mapping: convert to 0=fully open, 1=fully closed (consistent with leader arm)
                left_gripper_pos = left_gripper_pos / 255.0
                right_gripper_pos = right_gripper_pos / 255.0
            
            return left_gripper_pos, right_gripper_pos
        except Exception as e:
            self.get_logger().warn(f"Failed to get gripper positions: {e}")
            return 0.0, 0.0
    
    def get_joint_currents(self):
        """Calculate joint currents (based on torques)"""
        try:
            n_actuators = self.env.model.nu
            
            if n_actuators >= 16:
                # Calculate current from actuator force
                left_currents = self.env.data.actuator_force[:8].copy()  # Left arm 7 joints + gripper
                right_currents = self.env.data.actuator_force[8:16].copy()  # Right arm 7 joints + gripper
            elif n_actuators >= 8:
                left_currents = self.env.data.actuator_force[:8].copy()
                right_currents = np.zeros(8)
            else:
                left_currents = np.zeros(8)
                right_currents = np.zeros(8)
                if n_actuators > 0:
                    left_currents[:n_actuators] = self.env.data.actuator_force[:n_actuators]
            
            # Simple torque to current conversion (can be adjusted as needed)
            current_scale = 0.1  # Conversion factor
            left_currents *= current_scale
            right_currents *= current_scale
            
            return left_currents, right_currents
        except Exception as e:
            self.get_logger().warn(f"Failed to get joint currents: {e}")
            return np.zeros(8), np.zeros(8)
    
    def publish_states(self):
        """Publish robot states"""
        try:
            # Get joint angles
            left_joints, right_joints = self.get_joint_positions()
            
            # Get gripper positions
            left_gripper, right_gripper = self.get_gripper_positions()
            
            # Get joint currents
            left_currents, right_currents = self.get_joint_currents()
            
            # Debug info: print gripper status
            if hasattr(self, '_last_gripper_debug_time'):
                if time.time() - self._last_gripper_debug_time > 1.0:  # Print once per second
                    
                    self._last_gripper_debug_time = time.time()
            else:
                self._last_gripper_debug_time = time.time()
            
            # Publish left arm state
            left_joint_msg = Float64MultiArray()
            left_joint_msg.data = left_joints.tolist()
            self.left_joint_pub.publish(left_joint_msg)
            
            left_current_msg = Float64MultiArray()
            left_current_msg.data = left_currents.tolist()
            self.left_current_pub.publish(left_current_msg)
            
            left_gripper_msg = Float64MultiArray()
            left_gripper_msg.data = [left_gripper]
            self.left_gripper_pub.publish(left_gripper_msg)
            
            # Publish right arm state
            right_joint_msg = Float64MultiArray()
            right_joint_msg.data = right_joints.tolist()
            self.right_joint_pub.publish(right_joint_msg)
            
            right_current_msg = Float64MultiArray()
            right_current_msg.data = right_currents.tolist()
            self.right_current_pub.publish(right_current_msg)
            
            right_gripper_msg = Float64MultiArray()
            right_gripper_msg.data = [right_gripper]
            self.right_gripper_pub.publish(right_gripper_msg)
            
        except Exception as e:
            self.get_logger().warn(f"Failed to publish states: {e}")
    
    def destroy_node(self):
        """Clean up resources"""
        self.sim_running = False
        if self.sim_thread.is_alive():
            self.sim_thread.join(timeout=1.0)
        
        try:
            if hasattr(self, 'env'):
                self.env.close()
        except Exception as e:
            self.get_logger().warn(f"Failed to close environment: {e}")
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = XArmMujocoSimNode()
        node.get_logger().info("Simulation node started successfully, waiting for Gello commands...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Simulation node stopped")
    except Exception as e:
        print(f"Simulation node error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            if node is not None:
                node.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    
    
