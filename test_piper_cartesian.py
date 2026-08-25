#!/usr/bin/env python3
# -*-coding:utf8-*-
"""
笛卡尔空间运动控制示例（直线运动）
"""
import time
from piper_sdk import *

if __name__ == "__main__":
    print("正在初始化机械臂...")

    piper = C_PiperInterface_V2(can_name="can0", judge_flag=False)
    piper.ConnectPort()

    time.sleep(0.1)

    # 使能机械臂
    print("使能机械臂...")
    while not piper.EnablePiper():
        time.sleep(0.01)
    print("使能成功！")

    print("\n开始执行笛卡尔空间运动...")
    print("="*50)

    # 读取当前末端位姿
    end_pose = piper.GetArmEndPoseMsgs().end_pose
    print(f"\n当前末端位姿:")
    print(f"  X: {end_pose.X_axis / 1000:.3f} mm")
    print(f"  Y: {end_pose.Y_axis / 1000:.3f} mm")
    print(f"  Z: {end_pose.Z_axis / 1000:.3f} mm")
    print(f"  RX: {end_pose.RX_axis / 1000:.3f} 度")
    print(f"  RY: {end_pose.RY_axis / 1000:.3f} 度")
    print(f"  RZ: {end_pose.RZ_axis / 1000:.3f} 度")

    # 移动到初始位置 (使用MOVEP模式 - 点到点)
    print("\n1. 移动到初始位置...")
    piper.MotionCtrl_2(0x01, 0x00, 50, 0x00)  # MOVEP模式
    piper.EndPoseCtrl(150000, -50000, 150000, -179900, 0, -179900)
    time.sleep(3)

    end_pose = piper.GetArmEndPoseMsgs().end_pose
    print(f"  X: {end_pose.X_axis / 1000:.3f} mm")
    print(f"  Y: {end_pose.Y_axis / 1000:.3f} mm")
    print(f"  Z: {end_pose.Z_axis / 1000:.3f} mm")

    # 切换到MOVEL模式 (直线运动)
    print("\n2. 直线运动 - 沿Y轴移动100mm...")
    piper.MotionCtrl_2(0x01, 0x02, 50, 0x00)  # MOVEL模式
    piper.EndPoseCtrl(150000, 50000, 150000, -179900, 0, -179900)
    time.sleep(3)

    end_pose = piper.GetArmEndPoseMsgs().end_pose
    print(f"  Y: {end_pose.Y_axis / 1000:.3f} mm")

    # 沿X轴移动
    print("\n3. 直线运动 - 沿X轴移动100mm...")
    piper.MotionCtrl_2(0x01, 0x02, 50, 0x00)
    piper.EndPoseCtrl(250000, 50000, 150000, -179900, 0, -179900)
    time.sleep(3)

    end_pose = piper.GetArmEndPoseMsgs().end_pose
    print(f"  X: {end_pose.X_axis / 1000:.3f} mm")

    # 回到起点
    print("\n4. 回到起点...")
    piper.MotionCtrl_2(0x01, 0x02, 50, 0x00)
    piper.EndPoseCtrl(150000, -50000, 150000, -179900, 0, -179900)
    time.sleep(3)

    end_pose = piper.GetArmEndPoseMsgs().end_pose
    print(f"  X: {end_pose.X_axis / 1000:.3f} mm")
    print(f"  Y: {end_pose.Y_axis / 1000:.3f} mm")

    print("\n笛卡尔运动完成！")
    print("="*50)
