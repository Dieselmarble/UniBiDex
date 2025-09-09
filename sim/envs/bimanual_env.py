import numpy as np
import mujoco
import mujoco.viewer
try:
    from mujoco import mjx
    import jax.numpy as jnp
    HAS_JAX = True
except ImportError:
    HAS_JAX = False
    print("JAX not available, some features may be disabled")

import time
from typing import Dict, List, Tuple, Optional
try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError:
    import gym
    from gym import spaces
import os
from pathlib import Path

class BimanualXArm7Env(gym.Env):
    """
    MuJoCo environment for dual XArm-7 robots with grippers
    State: [left_arm_joints (7), left_gripper (1), right_arm_joints (7), right_gripper (1), 
            left_arm_velocities (7), left_gripper_vel (1), right_arm_velocities (7), right_gripper_vel (1)]
    Action: [left_arm_joint_commands (7), left_gripper (1), right_arm_joint_commands (7), right_gripper (1)]
    """
    
    def __init__(self, 
                 model_path: str = None,
                 obs_history_length: int = 4,
                 action_horizon: int = 10,
                 render_mode: str = None,
                 use_grippers: bool = True):
        super().__init__()
        
        self.obs_history_length = obs_history_length
        self.action_horizon = action_horizon
        self.render_mode = render_mode
        self.use_grippers = use_grippers
        
        # Load MuJoCo model
        if model_path is None:
            model_path = self._create_default_model()
        
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        
        # Define dimensions based on gripper usage
        if self.use_grippers:
            self.state_dim = 32  # 2 arms × 8 joints (7+1) × 2 (pos, vel)
            self.action_dim = 16  # 2 arms × 8 joints (7+1)
        else:
            self.state_dim = 28  # 2 arms × 7 joints × 2 (pos, vel)
            self.action_dim = 14  # 2 arms × 7 joints
        
        # Define observation and action spaces
        self.observation_space = spaces.Dict({
            'state_history': spaces.Box(
                low=-np.inf, high=np.inf, 
                shape=(self.obs_history_length, self.state_dim),
                dtype=np.float32
            ),
            'images': spaces.Dict({
                'front': spaces.Box(low=0, high=255, shape=(480, 640, 3), dtype=np.uint8),
                'left_wrist': spaces.Box(low=0, high=255, shape=(480, 640, 3), dtype=np.uint8),
                'right_wrist': spaces.Box(low=0, high=255, shape=(480, 640, 3), dtype=np.uint8),
            })
        })
        
        # XArm-7 joint limits (从XML文件中的ctrlrange提取) - 先定义关节限制
        self.left_joint_limits = np.array([
            [-6.28319, 6.28319],    # j1 (默认范围)
            [-2.059, 2.0944],       # j2  
            [-6.28319, 6.28319],    # j3 (默认范围)
            [-0.19198, 3.927],      # j4
            [-6.28319, 6.28319],    # j5 (默认范围)
            [-1.69297, 3.14159],    # j6
            [-6.28319, 6.28319],    # j7 (默认范围)
        ])
        
        self.right_joint_limits = self.left_joint_limits.copy()
        
        if self.use_grippers:
            # Gripper limits (从XML ctrlrange: 0-255)
            gripper_limits = np.array([[0.0, 255.0]])
            self.left_joint_limits = np.vstack([self.left_joint_limits, gripper_limits])
            self.right_joint_limits = np.vstack([self.right_joint_limits, gripper_limits])
        
        # Action space现在直接使用弧度范围 - 在关节限制定义后创建
        if self.use_grippers:
            # 包含夹爪：左臂7关节+夹爪1 + 右臂7关节+夹爪1
            action_low = np.concatenate([
                self.left_joint_limits[:, 0],   # 左臂+夹爪下限
                self.right_joint_limits[:, 0]   # 右臂+夹爪下限
            ])
            action_high = np.concatenate([
                self.left_joint_limits[:, 1],   # 左臂+夹爪上限
                self.right_joint_limits[:, 1]   # 右臂+夹爪上限
            ])
        else:
            # 仅关节：左臂7关节 + 右臂7关节
            action_low = np.concatenate([
                self.left_joint_limits[:7, 0],  # 左臂下限
                self.right_joint_limits[:7, 0]  # 右臂下限
            ])
            action_high = np.concatenate([
                self.left_joint_limits[:7, 1],  # 左臂上限
                self.right_joint_limits[:7, 1]  # 右臂上限
            ])
        
        self.action_space = spaces.Box(
            low=action_low, 
            high=action_high, 
            shape=(self.action_dim,),
            dtype=np.float32
        )
        
        # Initialize observation history
        self._state_history = np.zeros((self.obs_history_length, self.state_dim))
        
        # Initialize viewer
        self.viewer = None
        
        # Initialize renderer for image capture
        self.renderer = None
        
        # Camera settings
        self.camera_names = ['front_cam', 'left_wrist_cam', 'right_wrist_cam']
        self.camera_width = 640
        self.camera_height = 480
        

        
    def _create_default_model(self) -> str:
        """Get the path to the official dual XArm-7 model XML file"""
        # Get the directory where this script is located
        current_dir = Path(__file__).parent
        model_path = current_dir / "dual_xarm7_official.xml"
        
        if not model_path.exists():
            raise FileNotFoundError(f"Official MuJoCo model file not found: {model_path}")
        
        return str(model_path)
    

    
    def _get_state(self) -> np.ndarray:
        """Get the current state vector"""
        if self.use_grippers:
            # Get all joint positions and velocities (including grippers)
            joint_pos = self.data.qpos[:16]  # 7+1 per arm
            joint_vel = self.data.qvel[:16]  # 7+1 per arm
        else:
            # Get only arm joint positions and velocities
            joint_pos = np.concatenate([
                self.data.qpos[:7],   # left arm
                self.data.qpos[8:15]  # right arm (skip left gripper)
            ])
            joint_vel = np.concatenate([
                self.data.qvel[:7],   # left arm
                self.data.qvel[8:15]  # right arm (skip left gripper)
            ])
        
        return np.concatenate([joint_pos, joint_vel])
    
    def _get_images(self) -> Dict[str, np.ndarray]:
        """Capture images from all cameras"""
        images = {}
        
        # Check if we can initialize OpenGL renderer
        can_render = True
        if self.renderer is None:
            try:
                # Try to detect if we have a display
                import os
                if 'DISPLAY' not in os.environ:
                    can_render = False
                    # print("No DISPLAY environment variable, skipping image rendering")
                else:
                    self.renderer = mujoco.Renderer(self.model, height=self.camera_height, width=self.camera_width)
            except Exception as e:
                print(f"Warning: Cannot initialize renderer (likely no display): {e}")
                can_render = False
                self.renderer = None
        
        for i, cam_name in enumerate(['front', 'left_wrist', 'right_wrist']):
            # Default to black image
            img = np.zeros((self.camera_height, self.camera_width, 3), dtype=np.uint8)
            
            # Try to get camera image only if renderer is available
            if can_render and self.renderer is not None:
                try:
                    cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, self.camera_names[i])
                    self.renderer.update_scene(self.data, camera=cam_id)
                    img = self.renderer.render()
                except Exception as e:
                    print(f"Warning: Could not render camera {cam_name}: {e}")
                    pass
            
            images[cam_name] = img
        
        return images
    
    def _update_state_history(self, state: np.ndarray):
        """Update the state history buffer"""
        self._state_history[:-1] = self._state_history[1:]
        self._state_history[-1] = state
    
    def reset(self, seed=None, options=None) -> Tuple[Dict, Dict]:
        """Reset the environment"""
        super().reset(seed=seed)
        
        # Reset MuJoCo simulation
        mujoco.mj_resetData(self.model, self.data)
        
        # Set initial joint positions (使用XML中定义的初始角度)
        if self.use_grippers:
            # Left arm: 从图片中的角度转换而来 (弧度) + gripper
            self.data.qpos[:7] = [0, -0.22, 0, 0.873, 0.0105, 1.117, -0.0035]  
            # Right arm: 同样的角度 + gripper  
            self.data.qpos[15:22] = [0, -0.22, 0, 0.873, 0.0105, 1.117, -0.0035]
            # 设置夹爪初始位置为关闭状态 (0 = fully closed)
            self.data.qpos[7:15] = [0.0] * 8      # 左夹爪所有关节 - 初始关闭
            self.data.qpos[22:30] = [0.0] * 8     # 右夹爪所有关节 - 初始关闭
        else:
            # Just arm joints
            self.data.qpos[:7] = [0, -0.22, 0, 0.873, 0.0105, 1.117, -0.0035]  # left arm
            self.data.qpos[7:14] = [0, -0.22, 0, 0.873, 0.0105, 1.117, -0.0035]  # right arm
        
        # Step simulation to settle
        for _ in range(100):
            mujoco.mj_step(self.model, self.data)
        
        # Get initial state and images
        state = self._get_state()
        images = self._get_images()
        
        # Initialize state history
        self._state_history = np.tile(state, (self.obs_history_length, 1))
        
        observation = {
            'state_history': self._state_history.copy(),
            'images': images
        }
        
        info = {'success': False}
        
        return observation, info
    
    def step(self, action: np.ndarray) -> Tuple[Dict, float, bool, bool, Dict]:
        """Step the environment"""
        # 直接使用弧度值，不进行任何裁剪
        if self.use_grippers:
            left_action = action[:8]
            right_action = action[8:16]
        else:
            left_action = action[:7]
            right_action = action[7:14]
        
        # 直接使用原始动作值，不裁剪到关节限制
        left_scaled = left_action
        right_scaled = right_action
        
        # Apply actions to actuators
        if self.use_grippers:
            # Set joint controls for left arm (first 7 joints)
            self.data.ctrl[:7] = left_scaled[:7]
            # Set joint controls for right arm (first 7 joints)
            self.data.ctrl[8:15] = right_scaled[:7]
            
            # Set gripper controls using actuator IDs (tendon control)
            try:
                # Left gripper actuator
                left_gripper_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_gripper_act")
                # Right gripper actuator
                right_gripper_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_gripper_act")
                
                # Set gripper controls directly in the ctrl array using actuator indices
                if left_gripper_id >= 0 and right_gripper_id >= 0:
                    self.data.ctrl[left_gripper_id] = left_scaled[7]
                    self.data.ctrl[right_gripper_id] = right_scaled[7]
                else:
                    # Fallback to direct ctrl setting
                    self.data.ctrl[7] = left_scaled[7]
                    self.data.ctrl[15] = right_scaled[7]
                     
            except Exception as e:
                # Fallback to direct ctrl setting (may not work for tendon actuators)
                self.data.ctrl[7] = left_scaled[7]
                self.data.ctrl[15] = right_scaled[7]
        else:
            self.data.ctrl[:7] = left_scaled
            self.data.ctrl[7:14] = right_scaled
        
        # Step simulation
        for _ in range(5):  # Sub-steps for stability
            mujoco.mj_step(self.model, self.data)
        
        # Get new state and images
        state = self._get_state()
        images = self._get_images()
        self._update_state_history(state)
        
        observation = {
            'state_history': self._state_history.copy(),
            'images': images
        }
        
        # Simple reward (can be customized)
        reward = 0.0
        
        # Check if done (simple time limit)
        done = False
        truncated = False
        
        info = {'success': False}
        
        return observation, reward, done, truncated, info
    

    
    def render(self):
        """Render the environment"""
        if self.render_mode is None:
            return
        
        try:
            if self.viewer is None:
                # Launch MuJoCo viewer for local display
                self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
                print("MuJoCo viewer launched for local display")
            
            # Update viewer
            if self.viewer is not None:
                self.viewer.sync()
        except Exception as e:
            print(f"Warning: Cannot display viewer (likely no display available): {e}")
            # Fallback to no rendering
            self.render_mode = None
    
    def close(self):
        """Close the environment"""
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None
        
        if self.renderer is not None:
            try:
                self.renderer.close()
            except:
                pass
            self.renderer = None

            
# Test function
def test_bimanual_env():
    """Test the bimanual environment"""
    print("Testing BimanualXArm7Env...")
    
    # Create environment
    env = BimanualXArm7Env(use_grippers=True, render_mode='human')
    
    print(f"State dimension: {env.state_dim}")
    print(f"Action dimension: {env.action_dim}")
    print(f"Observation space: {env.observation_space}")
    print(f"Action space: {env.action_space}")
    
    # Reset environment
    obs, info = env.reset()
    print(f"Initial observation shapes:")
    print(f"  State history: {obs['state_history'].shape}")
    for cam_name, img in obs['images'].items():
        print(f"  {cam_name}: {img.shape}")
    

    
    # Test a few random steps
    for i in range(5000):
        # action = env.action_space.sample()
        # obs, reward, done, truncated, info = env.step(action)
        
        # Try to render
        env.render()
        
        # if i % 10 == 0:
        #     print(f"Step {i+1}: reward={reward}, done={done}")
        
        # Small delay to see animation if viewer is working
        import time
        time.sleep(0.01)
    
    env.close()
    print("Test completed successfully!")


if __name__ == "__main__":
    test_bimanual_env()
