#!/usr/bin/env python3
# -*-coding:utf8-*-
"""
机械臂关节控制示例
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

    # 角度转换因子：弧度转毫度 (1000*180/π)
    factor = 57295.7795

    print("\n开始执行关节运动...")
    print("="*50)

    # 移动到初始位置 (所有关节0度)
    print("1. 移动到初始位置 (全部0度)...")
    piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)  # 使能，位置模式，50%速度，关节空间
    piper.JointCtrl(0, 0, 0, 0, 0, 0)
    time.sleep(3)

    # 读取当前位置
    joint_msg = piper.GetArmJointMsgs()
    print(f"当前关节角度: {joint_msg}")

    # 示例：移动到指定位置 [0.2, 0.2, -0.2, 0.3, -0.2, 0.5] 弧度
    print("\n2. 移动各关节到指定位置...")
    position = [0.2, 0.2, -0.2, 0.3, -0.2, 0.5]  # 弧度
    joint_0 = round(position[0] * factor)
    joint_1 = round(position[1] * factor)
    joint_2 = round(position[2] * factor)
    joint_3 = round(position[3] * factor)
    joint_4 = round(position[4] * factor)
    joint_5 = round(position[5] * factor)

    piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)
    piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
    time.sleep(3)

    joint_msg = piper.GetArmJointMsgs()
    print(f"当前关节角度: {joint_msg}")
    print(f"目标位置(弧度): {position}")

    # 回到初始位置
    print("\n3. 回到初始位置...")
    piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)
    piper.JointCtrl(0, 0, 0, 0, 0, 0)
    time.sleep(3)

    joint_msg = piper.GetArmJointMsgs()
    print(f"当前关节角度: {joint_msg}")

    print("\n运动完成！")
    print("="*50)
