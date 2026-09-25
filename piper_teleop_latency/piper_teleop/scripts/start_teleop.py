#!/usr/bin/env python3
"""
快速启动脚本 - Piper遥操作
用于启动实时遥操作控制系统
"""

import os
import shlex
import subprocess
import sys
from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
ROS_SETUP = Path('/opt/ros/jazzy/setup.bash')
LOCAL_SETUP = WORKSPACE_ROOT / 'install' / 'setup.bash'


def _ros_shell_prefix():
    """Build shell commands that load the ROS environment."""
    commands = []
    if ROS_SETUP.exists():
        commands.append(f"source {shlex.quote(str(ROS_SETUP))}")
    if LOCAL_SETUP.exists():
        commands.append(f"source {shlex.quote(str(LOCAL_SETUP))}")
    return ' && '.join(commands)


def _run_ros_command(args, **kwargs):
    """Run a ROS command in a shell that has the workspace sourced."""
    command = shlex.join(args)
    prefix = _ros_shell_prefix()
    if prefix:
        command = f"{prefix} && {command}"
    kwargs.setdefault('cwd', str(WORKSPACE_ROOT))
    return subprocess.run(['bash', '-lc', command], **kwargs)


def print_banner():
    """打印横幅"""
    print("\n" + "="*70)
    print("  Piper 遥操作系统 - 实时控制")
    print("="*70 + "\n")


def print_instructions():
    """打印使用说明"""
    print("\n" + "="*70)
    print("📖 使用说明:")
    print("="*70)
    print("1. 确保 Pico VR 已连接（WiFi 或 USB）")
    print("2. 确保 Piper 机器人已通电并连接")
    print("3. 戴上 Pico 头显，握住右手柄")
    print("4. 按住侧面的 GRIP 键激活控制")
    print("5. 移动手柄，机械臂末端会跟随移动")
    print("6. 松开 GRIP 键暂停控制")
    print("7. 扣动 TRIGGER 键控制夹爪开合")
    print("8. Ctrl+C 退出系统")
    print("="*70 + "\n")


def start_teleop():
    """启动遥操作系统"""
    print("🚀 启动遥操作系统...\n")

    # 构建launch命令
    launch_cmd = [
        'ros2', 'launch',
        'piper_teleop', 'full_teleop.launch.py',
    ]

    print(f"执行命令: {' '.join(launch_cmd)}\n")

    try:
        _run_ros_command(launch_cmd)
    except KeyboardInterrupt:
        print("\n\n✅ 遥操作系统已停止")
    except Exception as e:
        print(f"\n\n❌ 错误: {e}")


def main():
    """主函数"""
    print_banner()
    print_instructions()

    # 询问是否继续
    try:
        response = input("准备启动遥操作系统。按Enter继续，Ctrl+C取消... ")
    except KeyboardInterrupt:
        print("\n\n❌ 已取消")
        sys.exit(0)

    # 启动系统
    start_teleop()


if __name__ == '__main__':
    main()
