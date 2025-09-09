import glob
import time
from dataclasses import dataclass
from typing import Optional, Tuple
import numpy as np
import tyro
from unibidex_core.agents.unibidex_agent import UniBiDexAgent
from unibidex_core.agents.unibidex_agent import UniBiDexAgent
from unibidex_core.data_utils.format_obs import save_frame
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray  # For publishing joint commands

def print_color(*args, color=None, attrs=(), **kwargs):
    import termcolor
    if len(args) > 0:
        args = tuple(termcolor.colored(arg, color=color, attrs=attrs) for arg in args)
    print(*args, **kwargs)

@dataclass
class Args:
    agent: str = "none"
    hz: int = 100
    start_joints: Optional[Tuple[float, ...]] = None

    mock: bool = False
    use_save_interface: bool = False
    data_dir: str = "~/bc_data"
    bimanual: bool = False
    verbose: bool = False

    # ROS communication configuration
    ros_state_topic:   str = "/unibidex/joint_states"


def main(args: Args):
    args.bimanual = True

    if not args.mock:
        rclpy.init()
        node = rclpy.create_node('unibidex_control_node')

        # 1) Initialize data containers
        obs_container = {'joints': None}
        current_container = {'left': None, 'right': None}

        # 2) Define all callbacks
        def state_cb(msg: JointState):
            obs_container['joints'] = np.array(msg.position)

        def left_cur_cb(msg: Float64MultiArray):
            current_container['left'] = np.array(msg.data)

        def right_cur_cb(msg: Float64MultiArray):
            current_container['right'] = np.array(msg.data)

        # 3) 创建 Publisher / Subscriber
        pub_left  = node.create_publisher(Float64MultiArray, 'unibidex/left/joint_command', 10)
        pub_right = node.create_publisher(Float64MultiArray, 'unibidex/right/joint_command', 10)

        node.create_subscription(
            JointState,
            args.ros_state_topic,
            state_cb,
            10
        )
        # 注意这里的话题名称要跟桥接节点发布的保持一致
        node.create_subscription(
            Float64MultiArray,
            'xarm/left/joint_current',
            left_cur_cb,
            10
        )
        node.create_subscription(
            Float64MultiArray,
            'xarm/right/joint_current',
            right_cur_cb,
            10
        )

        node.get_logger().info("Waiting for first joint_states message...")
        # （可选）等待 first state
        # while rclpy.ok() and obs_container['joints'] is None:
        #     rclpy.spin_once(node, timeout_sec=0.1)
        node.get_logger().info("Received joint_states, entering control loop.")

    # Agent 初始化与重置
    if args.bimanual:
        right_port = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAUZ35W-if00-port0"
        left_port  = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAJZ5RY-if00-port0"
        left_agent  = UniBiDexAgent(port=left_port,  start_joints=1)
        right_agent = UniBiDexAgent(port=right_port, start_joints=9)

        reset_joints_left  = np.deg2rad([0, -90, -90, -90,  90, 0, 0])
        reset_joints_right = np.deg2rad([0, -90,  90, -90, -90, 0, 0])
        reset_joints = np.concatenate([reset_joints_left, reset_joints_right])
    else:
        usb_ports = glob.glob("/dev/serial/by-id/*")
        if not usb_ports:
            raise RuntimeError("No unibidex port found—please plug in your UniBiDex device")
        unibidex_port = usb_ports[0]
        reset_joints = (
            np.deg2rad([0, -90, 90, -90, -90, 0, 0])
            if args.start_joints is None
            else np.array(args.start_joints)
        )
        agent = UniBiDexAgent(port=unibidex_port)

    # 偏置数组：7 轴 + 1 夹爪
    # 先等到第一帧电流进来，才能知道长度是 8
    while current_container['left'] is None or current_container['right'] is None:
        rclpy.spin_once(node, timeout_sec=1.0/args.hz)
    n_cur = current_container['left'].shape[0]  # 应该是 8
    xarm_offset_left  = np.zeros(n_cur)
    xarm_offset_right = np.zeros(n_cur)
    # 标定次数
    n_calib = int(args.hz * 1.0)  # 比如用 1 秒的数据
    node.get_logger().info(f"Calibrating no-load currents with {n_calib} samples…")
    for i in range(n_calib):
        rclpy.spin_once(node, timeout_sec=1.0/args.hz)
        # 一定已有数据
        xarm_offset_left  += current_container['left']
        xarm_offset_right += current_container['right']
    xarm_offset_left  /= n_calib
    xarm_offset_right /= n_calib
    node.get_logger().info(f"Calibration done. Offsets:\n"
                        f" left: {xarm_offset_left}\n"
                        f" right: {xarm_offset_right}")

    print_color("\nStart 🚀🚀🚀", color="green", attrs=("bold",))
    save_path = None
    start_time = time.time()
    
    try:
        # 主控制循环
        while rclpy.ok():
            elapsed = time.time() - start_time
            print_color(f"\rTime passed: {elapsed:.2f}s",
                        color="white", attrs=("bold",), end="", flush=True)

            # 主控制循环内，替换原来的 obs_l/obs_r 构造
            raw_l = current_container['left']
            raw_r = current_container['right']
            # 如果还没收到电流，就跳过
            if raw_l is None or raw_r is None:
                rclpy.spin_once(node, timeout_sec=1.0/args.hz)
                continue

            # 补偿空载电流
            comp_l = raw_l - xarm_offset_left  # shape (8,)
            comp_r = raw_r - xarm_offset_right

            # 可选：对补偿后的电流做截断，避免过大值
            # max_cur = 1.0  # 根据需要设置
            # comp_l = np.clip(comp_l, -max_cur, max_cur)
            # comp_r = np.clip(comp_r, -max_cur, max_cur)

            # 只把前 7 轴电流给 UniBiDexAgent，丢弃最后一个夹爪电流
            arm_n = left_agent._robot.num_dofs()  # 或直接写 7
            obs_l = {'currents': comp_l[:arm_n]}
            obs_r = {'currents': comp_r[:arm_n]}

            left_action = left_agent.act(obs_l)
            right_action = right_agent.act(obs_r)

            msg_l = Float64MultiArray()
            msg_l.data = left_action.tolist()
            pub_left.publish(msg_l)

            msg_r = Float64MultiArray()
            msg_r.data = right_action.tolist()
            pub_right.publish(msg_r)

            # 处理 ROS 回调
            rclpy.spin_once(node, timeout_sec=1.0 / args.hz)

    except KeyboardInterrupt:
        pass
        # logger.info("Shutting down…")
    finally:
        # ① 把力模式关掉
        left_agent._robot.set_torque_mode(False)
        right_agent._robot.set_torque_mode(False)
        # ② 关闭 driver，释放线程和串口
        left_agent._robot._driver.close()
        right_agent._robot._driver.close()
        # ③ 清理 ROS
        node.destroy_node()
        rclpy.shutdown()

    # 退出时清理
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main(tyro.cli(Args))
