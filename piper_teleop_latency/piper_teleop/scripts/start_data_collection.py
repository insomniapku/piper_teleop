#!/usr/bin/env python3
"""
快速启动脚本 - Pico数据采集
用于快速启动数据采集系统
"""

import os
import shlex
import subprocess
import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = Path(os.environ.get("PIPER_WORKSPACE", str(PACKAGE_ROOT.parents[1])))
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


def check_prerequisites():
    """检查前置条件"""
    print("🔍 检查前置条件...")

    issues = []

    # 检查ROS 2环境
    if 'ROS_DISTRO' in os.environ:
        print(f"✅ ROS 2环境: {os.environ['ROS_DISTRO']}")
    elif ROS_SETUP.exists():
        print("ℹ️  ROS 2环境未预加载，脚本将自动加载 /opt/ros/jazzy/setup.bash")
    else:
        issues.append("❌ ROS 2环境未设置，且未找到 /opt/ros/jazzy/setup.bash")
    # Check the Piper URDF configured by the user.
    urdf_path = os.environ.get("PIPER_URDF_PATH", "")
    if not os.path.isfile(urdf_path):
        issues.append("Set PIPER_URDF_PATH to an existing Piper URDF file before starting")
    else:
        print("Piper URDF file exists")

    # Check the output directory.
    output_dir = os.environ.get("PIPER_DEMO_DIR", str(Path.home() / "data" / "piper_demos"))
    if not os.path.exists(output_dir):
        print(f"Creating output directory: {output_dir}")
        os.makedirs(output_dir, exist_ok=True)
    else:
        print("Output directory exists")
        print(f"✅ 输出目录存在")

    # 检查piper_teleop包
    try:
        result = _run_ros_command(
            ['ros2', 'pkg', 'prefix', 'piper_teleop'],
            capture_output=True,
            text=True,
            timeout=5
        )
        if result.returncode == 0:
            print(f"✅ piper_teleop包已安装")
        else:
            issues.append("❌ piper_teleop包未找到。请先构建工作空间并确认 install/setup.bash 已生成")
    except Exception as e:
        issues.append(f"❌ 无法检查piper_teleop包: {e}")

    return issues


def print_banner():
    """打印横幅"""
    print("\n" + "="*70)
    print("  Pico VR 数据采集系统 - 快速启动脚本")
    print("="*70 + "\n")


def print_instructions():
    """打印使用说明"""
    print("\n" + "="*70)
    print("📖 使用说明:")
    print("="*70)
    print("1. 确保Pico VR已连接（WiFi或USB）")
    print("2. 确保Piper机器人已通电并连接")
    print("3. 戴上Pico头显，握住右手柄")
    print("4. 按下B键开始录制")
    print("5. 执行演示任务")
    print("6. 再次按下B键停止录制")
    print("7. Ctrl+C 退出系统")
    print("="*70 + "\n")


def start_data_collection():
    """启动数据采集系统"""
    print("🚀 启动数据采集系统...\n")

    # 构建launch命令
    output_dir = os.environ.get("PIPER_DEMO_DIR", str(Path.home() / "data" / "piper_demos"))
    urdf_path = os.environ.get("PIPER_URDF_PATH", "")
    launch_cmd = [
        'ros2', 'launch',
        'piper_teleop', 'data_collection.launch.py',
        f"output_dir:={output_dir}",
        f"urdf_path:={urdf_path}",
        'xr_mode:=sdk',  # 使用真实Pico VR设备
        'record_camera:=false',  # 默认不录制相机
    ]

    print(f"执行命令: {' '.join(launch_cmd)}\n")

    try:
        _run_ros_command(launch_cmd)
    except KeyboardInterrupt:
        print("\n\n✅ 数据采集系统已停止")
    except Exception as e:
        print(f"\n\n❌ 错误: {e}")


def main():
    """主函数"""
    print_banner()

    # 检查前置条件
    issues = check_prerequisites()

    if issues:
        print("\n⚠️  发现以下问题:\n")
        for issue in issues:
            print(f"  {issue}")
        print("\n请解决以上问题后重试。\n")
        sys.exit(1)

    print("\n✅ 所有前置条件满足!\n")

    # 打印使用说明
    print_instructions()

    # 询问是否继续
    try:
        response = input("准备启动数据采集系统。按Enter继续，Ctrl+C取消... ")
    except KeyboardInterrupt:
        print("\n\n❌ 已取消")
        sys.exit(0)

    # 启动系统
    start_data_collection()


if __name__ == '__main__':
    main()
