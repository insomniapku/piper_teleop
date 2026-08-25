#!/usr/bin/env python3
"""
数据可视化工具
用于检查采集的数据质量
"""

import h5py
import numpy as np
import matplotlib.pyplot as plt
import sys
import os


def load_episode(filename):
    """加载episode数据"""
    if not os.path.exists(filename):
        print(f"❌ 文件不存在: {filename}")
        sys.exit(1)

    with h5py.File(filename, 'r') as f:
        print(f"\n📊 数据集信息:")
        print(f"文件: {filename}")
        print(f"\n包含的数据:")
        for key in f.keys():
            shape = f[key].shape
            dtype = f[key].dtype
            print(f"  • {key:25s}: shape={shape}, dtype={dtype}")

        # 读取元数据
        if 'episode_number' in f.attrs:
            print(f"\n元数据:")
            print(f"  • Episode: {f.attrs['episode_number']}")
            print(f"  • Date: {f.attrs.get('date', 'N/A')}")
            print(f"  • Duration: {f.attrs.get('duration', 0):.2f}s")
            print(f"  • Control Rate: {f.attrs.get('control_rate', 'N/A')} Hz")

        # 加载数据
        data = {}
        for key in f.keys():
            data[key] = f[key][:]

    return data


def visualize_trajectory(data, output_file=None):
    """可视化轨迹"""
    timestamps = data.get('timestamps', None)
    joint_positions = data.get('joint_positions', None)
    ee_position = data.get('ee_position', None)

    if timestamps is None:
        print("❌ 缺少时间戳数据")
        return

    # 创建图表
    fig, axes = plt.subplots(3, 1, figsize=(14, 10))

    # 1. 关节位置
    if joint_positions is not None:
        axes[0].set_title('Joint Positions', fontsize=14, fontweight='bold')
        for i in range(joint_positions.shape[1]):
            axes[0].plot(timestamps, joint_positions[:, i], label=f'Joint {i+1}', linewidth=1.5)
        axes[0].set_xlabel('Time (s)')
        axes[0].set_ylabel('Position (rad)')
        axes[0].legend(loc='upper right', ncol=6)
        axes[0].grid(True, alpha=0.3)

    # 2. 末端位置
    if ee_position is not None:
        axes[1].set_title('End Effector Position', fontsize=14, fontweight='bold')
        axes[1].plot(timestamps, ee_position[:, 0], label='X', linewidth=2)
        axes[1].plot(timestamps, ee_position[:, 1], label='Y', linewidth=2)
        axes[1].plot(timestamps, ee_position[:, 2], label='Z', linewidth=2)
        axes[1].set_xlabel('Time (s)')
        axes[1].set_ylabel('Position (m)')
        axes[1].legend(loc='upper right')
        axes[1].grid(True, alpha=0.3)

    # 3. 关节速度（如果有）
    joint_velocities = data.get('joint_velocities', None)
    if joint_velocities is not None:
        axes[2].set_title('Joint Velocities', fontsize=14, fontweight='bold')
        for i in range(joint_velocities.shape[1]):
            axes[2].plot(timestamps, joint_velocities[:, i], label=f'Joint {i+1}', linewidth=1.5, alpha=0.7)
        axes[2].set_xlabel('Time (s)')
        axes[2].set_ylabel('Velocity (rad/s)')
        axes[2].legend(loc='upper right', ncol=6)
        axes[2].grid(True, alpha=0.3)

    plt.tight_layout()

    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"✅ 图表已保存: {output_file}")
    else:
        plt.show()


def check_data_quality(data):
    """检查数据质量"""
    print(f"\n🔍 数据质量检查:")

    issues = []

    # 检查时间戳
    timestamps = data.get('timestamps', None)
    if timestamps is not None:
        dt = np.diff(timestamps)
        avg_rate = 1.0 / np.mean(dt) if len(dt) > 0 else 0
        std_rate = np.std(1.0 / dt) if len(dt) > 0 else 0

        print(f"\n时间戳:")
        print(f"  • 采样点数: {len(timestamps)}")
        print(f"  • 平均采样率: {avg_rate:.2f} Hz")
        print(f"  • 采样率标准差: {std_rate:.2f} Hz")

        if std_rate > 5:
            issues.append("⚠️  采样率不稳定")

    # 检查关节位置
    joint_positions = data.get('joint_positions', None)
    if joint_positions is not None:
        # 关节限位 (from URDF)
        joint_limits_lower = np.array([-2.62, 0.0, -2.97, -1.75, -1.22, -2.09])
        joint_limits_upper = np.array([2.62, 3.14, 0.0, 1.75, 1.22, 2.09])

        print(f"\n关节位置:")
        for i in range(joint_positions.shape[1]):
            pos = joint_positions[:, i]
            print(f"  • Joint {i+1}:")
            print(f"    - 范围: [{np.min(pos):.3f}, {np.max(pos):.3f}] rad")
            print(f"    - 平均: {np.mean(pos):.3f} rad")

            # 检查限位
            if np.any(pos < joint_limits_lower[i]) or np.any(pos > joint_limits_upper[i]):
                issues.append(f"⚠️  Joint {i+1} 超出限位")

        # 检查NaN
        if np.any(np.isnan(joint_positions)):
            issues.append("❌ 关节位置包含NaN值")

    # 检查关节速度
    joint_velocities = data.get('joint_velocities', None)
    if joint_velocities is not None:
        max_vel = np.max(np.abs(joint_velocities))
        print(f"\n关节速度:")
        print(f"  • 最大速度: {max_vel:.3f} rad/s")

        if max_vel > 5.0:
            issues.append("⚠️  关节速度过大 (> 5 rad/s)")

    # 检查末端位置
    ee_position = data.get('ee_position', None)
    if ee_position is not None:
        print(f"\n末端位置:")
        for i, axis in enumerate(['X', 'Y', 'Z']):
            pos = ee_position[:, i]
            print(f"  • {axis}: [{np.min(pos):.3f}, {np.max(pos):.3f}] m")

        # 检查轨迹平滑度
        if len(ee_position) > 1:
            ee_vel = np.diff(ee_position, axis=0)
            ee_speed = np.linalg.norm(ee_vel, axis=1)
            max_speed = np.max(ee_speed) * avg_rate  # 转换为 m/s

            print(f"  • 最大速度: {max_speed:.3f} m/s")

            if max_speed > 0.5:
                issues.append("⚠️  末端速度过大 (> 0.5 m/s)")

    # 总结
    print(f"\n{'='*60}")
    if issues:
        print(f"⚠️  发现 {len(issues)} 个问题:")
        for issue in issues:
            print(f"  {issue}")
    else:
        print(f"✅ 数据质量良好，未发现问题")
    print(f"{'='*60}\n")

    return len(issues) == 0


def main():
    """主函数"""
    if len(sys.argv) < 2:
        print("用法: python3 visualize_data.py <episode_file.h5>")
        print("示例: python3 visualize_data.py episode_001.h5")
        sys.exit(1)

    filename = sys.argv[1]

    print("="*60)
    print("  数据可视化工具")
    print("="*60)

    # 加载数据
    data = load_episode(filename)

    # 质量检查
    is_good = check_data_quality(data)

    # 可视化
    output_file = filename.replace('.h5', '_visualization.png')
    visualize_trajectory(data, output_file)

    if is_good:
        print("\n✅ 数据可用于训练")
    else:
        print("\n⚠️  建议检查数据或重新采集")


if __name__ == '__main__':
    main()
