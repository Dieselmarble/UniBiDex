#!/usr/bin/env python3
# filepath: banana_learning/banana_learning/ours/envs/xarm_mujoco_sim.py

# 设置环境变量以获得更好的显示兼容性
import os
# 如果遇到GLX错误，可以尝试以下设置：
# os.environ['MUJOCO_GL'] = 'egl'  # 或者 'osmesa' 用于软件渲染
# os.environ['PYOPENGL_PLATFORM'] = 'egl'

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import threading
import time
import sys
from pathlib import Path

# 添加环境路径
env_path = Path(__file__).parent
sys.path.insert(0, str(env_path))

from bimanual_env import BimanualXArm7Env
from pathlib import Path

# 添加环境路径
env_path = Path(__file__).parent.parent / "banana_learning/banana_learning/ours/envs"
sys.path.insert(0, str(env_path))

from bimanual_env import BimanualXArm7Env

class XArmMujocoSimNode(Node):
    """
    基于BimanualXArm7Env的XArm双臂MuJoCo仿真节点
    订阅: /gello/left/joint_command, /gello/right/joint_command
    发布: /xarm/left/joint_angle, /xarm/left/joint_current, /xarm/left/gripper_opening
          /xarm/right/joint_angle, /xarm/right/joint_current, /xarm/right/gripper_opening
    """
    
    def __init__(self):
        super().__init__('xarm_mujoco_sim_node')
        
        # 初始化MuJoCo环境
        self.init_env()
        
        # 创建订阅者
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
        
        # 创建发布者
        self.left_joint_pub = self.create_publisher(Float64MultiArray, '/xarm/left/joint_angle', 10)
        self.left_current_pub = self.create_publisher(Float64MultiArray, '/xarm/left/joint_current', 10)
        self.left_gripper_pub = self.create_publisher(Float64MultiArray, '/xarm/left/gripper_opening', 10)
        
        self.right_joint_pub = self.create_publisher(Float64MultiArray, '/xarm/right/joint_angle', 10)
        self.right_current_pub = self.create_publisher(Float64MultiArray, '/xarm/right/joint_current', 10)
        self.right_gripper_pub = self.create_publisher(Float64MultiArray, '/xarm/right/gripper_opening', 10)
        
        # 控制命令缓存
        self.left_command = np.zeros(8)  # 7关节 + 1夹爪
        self.right_command = np.zeros(8)
        self.command_lock = threading.Lock()
        
        # 启动仿真线程
        self.sim_thread = threading.Thread(target=self.simulation_loop, daemon=True)
        self.sim_running = True
        self.sim_thread.start()
        
        # 状态发布定时器 (50Hz)
        self.state_timer = self.create_timer(0.02, self.publish_states)
        
        self.get_logger().info("XArm MuJoCo仿真节点已启动")
    
    def init_env(self):
        """初始化MuJoCo环境"""
        try:
            # 创建BimanualXArm7环境 (启用可视化)
            self.env = BimanualXArm7Env(
                use_grippers=True,
                render_mode='human'  # 启用可视化
            )
            
            # 重置环境
            obs, info = self.env.reset()
            
            self.get_logger().info(f"MuJoCo环境初始化成功")
            self.get_logger().info(f"状态维度: {self.env.state_dim}")
            self.get_logger().info(f"动作维度: {self.env.action_dim}")
            self.get_logger().info(f"模型qpos维度: {self.env.model.nq}")
            self.get_logger().info(f"模型actuator维度: {self.env.model.nu}")
            
        except Exception as e:
            self.get_logger().error(f"环境初始化失败: {e}")
            # 如果可视化失败，尝试无头模式
            try:
                self.get_logger().warn("尝试无头模式...")
                self.env = BimanualXArm7Env(
                    use_grippers=True,
                    render_mode=None  # 无头模式
                )
                obs, info = self.env.reset()
                self.get_logger().info("无头模式初始化成功")
            except Exception as e2:
                self.get_logger().error(f"无头模式也失败: {e2}")
                raise
    
    def left_command_callback(self, msg):
        """左臂命令回调"""
        with self.command_lock:
            if len(msg.data) >= 7:
                # 确保数组长度为8，如果只有7个关节，夹爪保持当前值
                self.left_command[:len(msg.data)] = msg.data[:8]
                if len(msg.data) == 7:
                    # 如果只有7个关节，夹爪设为中间位置
                    self.left_command[7] = 127.5  # 夹爪中间位置 (0-255范围)
                elif len(msg.data) == 8:
                    # 如果有8个值（包含夹爪），将夹爪值从0-1范围映射到0-255范围
                    # 注意：leader arm的gr_val=0表示fully open, gr_val=1表示fully closed
                    # MuJoCo的0=fully closed, 255=fully open
                    gripper_val = msg.data[7]  # 0-1 range from leader arm
                    # 修正映射：leader的1(closed) -> MuJoCo的255(closed), leader的0(open) -> MuJoCo的0(open)
                    self.left_command[7] = gripper_val * 255.0
        

    
    def right_command_callback(self, msg):
        """右臂命令回调"""
        with self.command_lock:
            if len(msg.data) >= 7:
                # 确保数组长度为8，如果只有7个关节，夹爪保持当前值
                self.right_command[:len(msg.data)] = msg.data[:8]
                if len(msg.data) == 7:
                    # 如果只有7个关节，夹爪设为中间位置
                    self.right_command[7] = 127.5  # 夹爪中间位置
                elif len(msg.data) == 8:
                    # 如果有8个值（包含夹爪），将夹爪值从0-1范围映射到0-255范围
                    # 注意：leader arm的gr_val=0表示fully open, gr_val=1表示fully closed
                    # MuJoCo的0=fully closed, 255=fully open
                    gripper_val = msg.data[7]  # 0-1 range from leader arm
                    # 修正映射：leader的1(closed) -> MuJoCo的255(closed), leader的0(open) -> MuJoCo的0(open)
                    self.right_command[7] = gripper_val * 255.0
        

    
    def simulation_loop(self):
        """仿真主循环 (100Hz)"""
        dt = 0.01  # 10ms
        
        while self.sim_running:
            start_time = time.time()
            
            # 构建完整的动作向量
            with self.command_lock:
                # 组合左臂和右臂命令 [left_7joints, left_gripper, right_7joints, right_gripper]
                action = np.concatenate([
                    self.left_command,   # 左臂7关节 + 夹爪
                    self.right_command   # 右臂7关节 + 夹爪
                ])
            
            # 执行环境步进
            try:
                obs, reward, done, truncated, info = self.env.step(action)
                
                # 渲染显示
                self.env.render()
                
            except Exception as e:
                self.get_logger().warn(f"仿真步进错误: {e}")
            
            # 控制仿真频率
            elapsed = time.time() - start_time
            sleep_time = max(0, dt - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    def get_joint_positions(self):
        """获取关节位置"""
        try:
            # 从环境获取关节位置
            if self.env.use_grippers:
                # 左臂7关节
                left_joints = self.env.data.qpos[:8]
                # 右臂7关节  
                right_joints = self.env.data.qpos[9:16]
            else:
                left_joints = self.env.data.qpos[:7]
                right_joints = self.env.data.qpos[7:14]
            
            return left_joints, right_joints
        except Exception as e:
            self.get_logger().warn(f"获取关节位置失败: {e}")
            return np.zeros(7), np.zeros(7)
    
    def get_gripper_positions(self):
        """获取夹爪位置"""
        try:
            # 默认夹爪位置
            left_gripper_pos = 0.0
            right_gripper_pos = 0.0
            
            # 从环境获取夹爪位置
            if self.env.use_grippers:
                n_qpos = self.env.model.nq
                if n_qpos > 7:
                    left_gripper_pos = self.env.data.qpos[7]  # 左夹爪
                if n_qpos > 15:
                    right_gripper_pos = self.env.data.qpos[15]  # 右夹爪
                
                # 将夹爪位置从MuJoCo的0-255范围转换回0-1范围用于发布
                # MuJoCo的0=fully closed, 255=fully open
                # 修正映射：转换为0=fully open, 1=fully closed (与leader arm一致)
                left_gripper_pos = left_gripper_pos / 255.0
                right_gripper_pos = right_gripper_pos / 255.0
            
            return left_gripper_pos, right_gripper_pos
        except Exception as e:
            self.get_logger().warn(f"获取夹爪位置失败: {e}")
            return 0.0, 0.0
    
    def get_joint_currents(self):
        """计算关节电流 (基于力矩)"""
        try:
            n_actuators = self.env.model.nu
            
            if n_actuators >= 16:
                # 从actuator force计算电流
                left_currents = self.env.data.actuator_force[:8].copy()  # 左臂7关节+夹爪
                right_currents = self.env.data.actuator_force[8:16].copy()  # 右臂7关节+夹爪
            elif n_actuators >= 8:
                left_currents = self.env.data.actuator_force[:8].copy()
                right_currents = np.zeros(8)
            else:
                left_currents = np.zeros(8)
                right_currents = np.zeros(8)
                if n_actuators > 0:
                    left_currents[:n_actuators] = self.env.data.actuator_force[:n_actuators]
            
            # 简单的力矩到电流转换 (可根据实际需要调整)
            current_scale = 0.1  # 转换系数
            left_currents *= current_scale
            right_currents *= current_scale
            
            return left_currents, right_currents
        except Exception as e:
            self.get_logger().warn(f"获取关节电流失败: {e}")
            return np.zeros(8), np.zeros(8)
    
    def publish_states(self):
        """发布机器人状态"""
        try:
            # 获取关节角度
            left_joints, right_joints = self.get_joint_positions()
            
            # 获取夹爪位置
            left_gripper, right_gripper = self.get_gripper_positions()
            
            # 获取关节电流
            left_currents, right_currents = self.get_joint_currents()
            
            # 调试信息：打印夹爪状态
            if hasattr(self, '_last_gripper_debug_time'):
                if time.time() - self._last_gripper_debug_time > 1.0:  # 每秒打印一次
                    
                    self._last_gripper_debug_time = time.time()
            else:
                self._last_gripper_debug_time = time.time()
            
            # 发布左臂状态
            left_joint_msg = Float64MultiArray()
            left_joint_msg.data = left_joints.tolist()
            self.left_joint_pub.publish(left_joint_msg)
            
            left_current_msg = Float64MultiArray()
            left_current_msg.data = left_currents.tolist()
            self.left_current_pub.publish(left_current_msg)
            
            left_gripper_msg = Float64MultiArray()
            left_gripper_msg.data = [left_gripper]
            self.left_gripper_pub.publish(left_gripper_msg)
            
            # 发布右臂状态
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
            self.get_logger().warn(f"发布状态失败: {e}")
    
    def destroy_node(self):
        """清理资源"""
        self.sim_running = False
        if self.sim_thread.is_alive():
            self.sim_thread.join(timeout=1.0)
        
        try:
            if hasattr(self, 'env'):
                self.env.close()
        except Exception as e:
            self.get_logger().warn(f"关闭环境失败: {e}")
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = XArmMujocoSimNode()
        node.get_logger().info("仿真节点启动成功，等待Gello命令...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("仿真节点已停止")
    except Exception as e:
        print(f"仿真节点错误: {e}")
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
    
    
