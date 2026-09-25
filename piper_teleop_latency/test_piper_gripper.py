#!/usr/bin/env python3
# -*-coding:utf8-*-
"""
夹爪控制示例
"""
import time
from piper_sdk import *

if __name__ == "__main__":
    print("正在初始化机械臂...")

    piper = C_PiperInterface_V2(can_name="can0", judge_flag=False)
    piper.ConnectPort()

    time.sleep(0.1)

    print("\n夹爪控制测试")
    print("="*50)

    # 读取夹爪状态
    print("读取夹爪状态...")
    gripper_status = piper.GetArmGripperMsgs()
    print(f"夹爪状态: {gripper_status}")

    # 使能机械臂和夹爪
    print("\n使能机械臂...")
    while not piper.EnablePiper():
        time.sleep(0.01)

    print("配置夹爪参数...")
    piper.GripperCtrl(0, 1000, 0x02, 0)  # 回零
    time.sleep(1)
    piper.GripperCtrl(0, 1000, 0x01, 0)  # 使能
    time.sleep(1)

    # 打开夹爪 (50mm = 50000微米)
    print("\n打开夹爪到50mm...")
    piper.GripperCtrl(50000, 1000, 0x01, 0)  # 位置50000微米，速度1000
    time.sleep(2)

    gripper_status = piper.GetArmGripperMsgs()
    print(f"夹爪状态: {gripper_status}")

    # 关闭夹爪
    print("\n关闭夹爪...")
    piper.GripperCtrl(0, 1000, 0x01, 0)  # 位置0 (关闭)
    time.sleep(2)

    gripper_status = piper.GetArmGripperMsgs()
    print(f"夹爪状态: {gripper_status}")

    # 半开夹爪 (25mm)
    print("\n半开夹爪到25mm...")
    piper.GripperCtrl(25000, 1000, 0x01, 0)  # 位置25000微米
    time.sleep(2)

    gripper_status = piper.GetArmGripperMsgs()
    print(f"夹爪状态: {gripper_status}")

    print("\n夹爪测试完成！")
    print("="*50)
