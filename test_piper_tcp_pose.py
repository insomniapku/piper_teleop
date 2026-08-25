#!/usr/bin/env python3
# -*-coding:utf8-*-
"""
读取机械臂末端位姿（TCP位置）
"""
import time
from piper_sdk import *

if __name__ == "__main__":
    print("正在初始化机械臂连接...")

    piper = C_PiperInterface_V2(can_name="can0", judge_flag=False)
    piper.ConnectPort()

    time.sleep(0.1)

    print("="*50)
    print("开始读取末端位姿（Ctrl+C退出）...")
    print("="*50)

    try:
        while True:
            end_pose_msg = piper.GetArmEndPoseMsgs()
            pose = end_pose_msg.end_pose

            print(f"\n末端位姿:")
            print(f"  X: {pose.X_axis / 1000:.3f} mm")
            print(f"  Y: {pose.Y_axis / 1000:.3f} mm")
            print(f"  Z: {pose.Z_axis / 1000:.3f} mm")
            print(f"  RX: {pose.RX_axis / 1000:.3f} 度")
            print(f"  RY: {pose.RY_axis / 1000:.3f} 度")
            print(f"  RZ: {pose.RZ_axis / 1000:.3f} 度")

            time.sleep(0.5)  # 2Hz刷新
    except KeyboardInterrupt:
        print("\n\n程序已停止")
