#!/usr/bin/env python3
# -*-coding:utf8-*-
"""
Piper机械臂完整功能演示
包含：使能、关节运动、笛卡尔运动、夹爪控制
"""
import time
from piper_sdk import *

def print_section(title):
    """打印分隔线"""
    print("\n" + "="*60)
    print(f"  {title}")
    print("="*60)

def main():
    print_section("初始化机械臂")

    # 初始化连接
    piper = C_PiperInterface_V2(can_name="can0", judge_flag=False)
    piper.ConnectPort()
    time.sleep(0.1)

    # 读取固件版本
    firmware = piper.GetPiperFirmwareVersion()
    print(f"固件版本: {firmware}")

    # 使能机械臂
    print("使能机械臂中...")
    while not piper.EnablePiper():
        time.sleep(0.01)
    print("✓ 使能成功")

    # ========== 1. 读取当前状态 ==========
    print_section("1. 读取当前状态")

    # 读取关节角度
    joint_msg = piper.GetArmJointMsgs()
    print("关节角度:")
    print(f"  {joint_msg}")

    # 读取末端位姿
    end_pose = piper.GetArmEndPoseMsgs().end_pose
    print("\n末端位姿:")
    print(f"  位置 (mm): X={end_pose.X_axis/1000:.1f}, Y={end_pose.Y_axis/1000:.1f}, Z={end_pose.Z_axis/1000:.1f}")
    print(f"  姿态 (度): RX={end_pose.RX_axis/1000:.1f}, RY={end_pose.RY_axis/1000:.1f}, RZ={end_pose.RZ_axis/1000:.1f}")

    # 读取夹爪状态
    gripper = piper.GetArmGripperMsgs()
    print(f"\n夹爪状态: {gripper.gripper_state}")

    # ========== 2. 关节空间运动 ==========
    print_section("2. 关节空间运动演示")

    factor = 57295.7795  # 弧度转毫度

    print("移动到初始位置 [0, 0, 0, 0, 0, 0]...")
    piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)
    piper.JointCtrl(0, 0, 0, 0, 0, 0)
    time.sleep(3)
    print("✓ 完成")

    print("\n移动到位置 [0.1, 0.1, -0.1, 0.2, -0.1, 0.3] 弧度...")
    position = [0.1, 0.1, -0.1, 0.2, -0.1, 0.3]
    joints = [round(p * factor) for p in position]
    piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)
    piper.JointCtrl(*joints)
    time.sleep(3)
    print("✓ 完成")

    # ========== 3. 笛卡尔空间运动 ==========
    print_section("3. 笛卡尔空间运动演示")

    print("移动到工作空间中心位置...")
    piper.MotionCtrl_2(0x01, 0x00, 50, 0x00)  # MOVEP模式
    piper.EndPoseCtrl(200000, 0, 200000, -179900, 0, -179900)
    time.sleep(3)
    print("✓ 完成")

    print("\n画一个小正方形 (边长50mm)...")
    piper.MotionCtrl_2(0x01, 0x02, 50, 0x00)  # MOVEL模式 - 直线运动

    # 正方形四个顶点
    points = [
        (200000, 25000, 200000),   # 点1
        (200000, -25000, 200000),  # 点2
        (250000, -25000, 200000),  # 点3
        (250000, 25000, 200000),   # 点4
        (200000, 25000, 200000),   # 回到点1
    ]

    for i, (x, y, z) in enumerate(points, 1):
        print(f"  移动到点 {i}...")
        piper.EndPoseCtrl(x, y, z, -179900, 0, -179900)
        time.sleep(2)

    print("✓ 正方形完成")

    # ========== 4. 夹爪控制 ==========
    print_section("4. 夹爪控制演示")

    print("配置夹爪...")
    piper.GripperCtrl(0, 1000, 0x02, 0)  # 回零
    time.sleep(1)
    piper.GripperCtrl(0, 1000, 0x01, 0)  # 使能
    time.sleep(1)

    print("\n打开夹爪 (40mm)...")
    piper.GripperCtrl(40000, 1000, 0x01, 0)
    time.sleep(2)
    gripper = piper.GetArmGripperMsgs()
    print(f"  当前状态: {gripper.gripper_state}")

    print("\n关闭夹爪...")
    piper.GripperCtrl(0, 1000, 0x01, 0)
    time.sleep(2)
    gripper = piper.GetArmGripperMsgs()
    print(f"  当前状态: {gripper.gripper_state}")

    print("\n半开夹爪 (20mm)...")
    piper.GripperCtrl(20000, 1000, 0x01, 0)
    time.sleep(2)
    gripper = piper.GetArmGripperMsgs()
    print(f"  当前状态: {gripper.gripper_state}")

    # ========== 5. 回到初始位置 ==========
    print_section("5. 回到初始位置")

    print("关节归零...")
    piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)
    piper.JointCtrl(0, 0, 0, 0, 0, 0)
    time.sleep(3)
    print("✓ 完成")

    print_section("演示完成！")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n程序被用户中断")
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()
