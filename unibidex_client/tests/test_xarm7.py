#!/usr/bin/env python3

import time
from unibidex_client.motion_control.xarm7 import XArm7_Hand

def main():
    # 1. Replace the IP/serial port below with your robot arm's network address
    ARM_IP = "192.168.100.203"
    HAND_TTY = "/dev/ttyUSB0"   # This field can be left empty if only controlling the robot arm

    # 2. 初始化 XArm7Ability，只打开机械臂控制
    arm = XArm7_Hand(
        use_arm=True,
        use_hand=False,
        arm_ip=ARM_IP,
        hand_tty_index=HAND_TTY,
        is_right=False,          # 左臂 False，右臂 True
        use_servo_control=True, # 如果想走快速模式可以改成 True
    )
    arm.reset()  # 重置
    arm.start()  # 启动控制线程

    try:
        
        arm.get_arm_qpos()  # 获取当前关节角度
        arm.get_arm_ee_pose()  # 获取末端执行器位置
        arm.move_ee_to([0.445, 0.0, 0.50])
        # # 3. 移动到示例目标关节角度（弧度制，共 7 维）
        # print("Moving to target pose...")
        # target_qpos = [0.0, 0.5, 0.0, -0.5, 0.0, 0.3, 0.0]
        # arm.control_arm_qpos(target_qpos)
        # time.sleep(3.0)
        init_qpos = arm.get_arm_qpos()            # current 7-joint angles
        steps     = 1000
        radius    = 0.15
        duration  = 10.0

        # returns (N×7 qpos, N×3 ee_positions)
        qpos_list, ee_pos_list = arm.generate_circle_motion(
            radius=radius,
            init_qpos=init_qpos,
            steps=steps
        )

        # stream it out at constant rate
        dt = duration / steps
        for q in qpos_list:
            arm.control_arm_qpos(q)
            time.sleep(dt)
         
        # 4. 回到初始位姿（全部 0）
        # print("Returning to home pose...")
        # arm.control_arm_qpos([0.1] * 7)
        # time.sleep(3.0)

    except KeyboardInterrupt:
        print("Interrupted by user")

    finally:
        # 5. 停止并清理
        print("Stopping arm...")
        arm.stop()
        print("Done.")

if __name__ == "__main__":
    main()