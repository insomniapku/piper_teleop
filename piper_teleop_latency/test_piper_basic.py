#!/usr/bin/env python3
# -*-coding:utf8-*-
"""
基础测试脚本 - 读取机械臂关节角度和固件版本
"""
import time
from piper_sdk import *

if __name__ == "__main__":
    # 实例化interface
    # can_name: CAN端口名称，默认为'can0'
    # judge_flag: 是否判断是否为官方CAN模块，如果使用非官方模块请设为False
    # can_auto_init: 是否自动初始化CAN总线
    piper = C_PiperInterface(
        can_name="can0",
        judge_flag=False,  # 如果不是官方CAN模块，设为False
        can_auto_init=True,
        dh_is_offset=1,  # 使用新版DH参数（S-V1.6-3及以后）
        start_sdk_joint_limit=False,
        start_sdk_gripper_limit=False,
        logger_level=LogLevel.WARNING,
        log_to_file=False,
        log_file_path=None
    )

    # 连接端口，开启CAN收发线程
    piper.ConnectPort()

    # 等待一下让设备初始化
    time.sleep(0.1)

    # 读取固件版本
    print("="*50)
    print("正在读取机械臂固件版本...")
    firmware_version = piper.GetPiperFirmwareVersion()
    print(f"固件版本: {firmware_version}")
    print("="*50)

    # 循环读取关节角度（需要机械臂处于从臂模式）
    print("\n开始读取关节角度（Ctrl+C退出）...")
    print("注意：读取关节反馈需要机械臂处在从臂模式下")
    print("="*50)

    try:
        while True:
            joint_msgs = piper.GetArmJointMsgs()
            print(f"关节角度: {joint_msgs}")
            time.sleep(0.1)  # 10Hz刷新
    except KeyboardInterrupt:
        print("\n程序已停止")
