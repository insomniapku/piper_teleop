#!/usr/bin/env python3
# -*-coding:utf8-*-
"""
切换机械臂到从臂模式
"""
from piper_sdk import *

if __name__ == "__main__":
    print("正在切换机械臂到从臂模式...")

    piper = C_PiperInterface(can_name="can0", judge_flag=False)
    piper.ConnectPort()

    # 切换到从臂模式
    # MasterSlaveConfig(0xFC, 0, 0, 0) - 切换到从臂
    piper.MasterSlaveConfig(0xFC, 0, 0, 0)

    print("已发送切换到从臂模式的指令")
    print("现在可以运行 test_piper_basic.py 来读取关节角度")
