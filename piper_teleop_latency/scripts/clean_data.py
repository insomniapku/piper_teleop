#!/usr/bin/env python3
"""
数据清洗工具
移除异常值，平滑轨迹
"""

import h5py
import numpy as np
import sys
import os
from scipy.signal import savgol_filter


def clean_episode(input_file, output_file=None):
    """清洗episode数据"""

    if output_file is None:
        output_file = input_file.replace('.h5', '_clean.h5')

    print(f"📂 输入文件: {input_file}")
    print(f"📂 输出文件: {output_file}")

    with h5py.File(input_file, 'r') as f_in:
        # 读取数据
        timestamps = f_in['timestamps'][:]
        joint_positions = f_in['joint_positions'][:]

        original_length = len(timestamps)
        print(f"\n原始数据: {original_length} 个样本")

        # 1. 移除NaN值
        valid_mask = ~np.any(np.isnan(joint_positions), axis=1)
        nan_removed = original_length - np.sum(valid_mask)
        if nan_removed > 0:
            print(f"  • 移除 {nan_removed} 个NaN样本")

        # 2. 检查关节限位
        joint_limits_lower = np.array([-2.62, 0.0, -2.97, -1.75, -1.22, -2.09])
        joint_limits_upper = np.array([2.62, 3.14, 0.0, 1.75, 1.22, 2.09])

        limits_mask = np.all(
            (joint_positions >= joint_limits_lower - 0.01) &
            (joint_positions <= joint_limits_upper + 0.01),
            axis=1
        )
        limits_removed = np.sum(valid_mask) - np.sum(valid_mask & limits_mask)
        if limits_removed > 0:
            print(f"  • 移除 {limits_removed} 个超限样本")

        valid_mask = valid_mask & limits_mask

        # 3. 移除速度异常点
        if len(joint_positions) > 1:
            dt = np.diff(timestamps)
            joint_vel = np.diff(joint_positions, axis=0) / dt[:, None]
            max_joint_vel = np.max(np.abs(joint_vel), axis=1)

            vel_mask = np.ones(len(joint_positions), dtype=bool)
            vel_mask[1:] = max_joint_vel < 5.0  # 5 rad/s 限制

            vel_removed = np.sum(valid_mask) - np.sum(valid_mask & vel_mask)
            if vel_removed > 0:
                print(f"  • 移除 {vel_removed} 个速度异常样本")

            valid_mask = valid_mask & vel_mask

        clean_length = np.sum(valid_mask)
        print(f"\n清洗后数据: {clean_length} 个样本")
        print(f"保留率: {clean_length/original_length*100:.1f}%")

        # 4. 平滑轨迹（可选）
        if clean_length > 51:  # 需要足够的点进行平滑
            print(f"\n应用Savitzky-Golay滤波器平滑轨迹...")
            joint_positions_clean = joint_positions[valid_mask]
            joint_positions_smooth = savgol_filter(joint_positions_clean, window_length=11, polyorder=3, axis=0)
        else:
            joint_positions_smooth = joint_positions[valid_mask]

        # 保存清洗后的数据
        with h5py.File(output_file, 'w') as f_out:
            # 保存清洗后的数据
            f_out.create_dataset('timestamps', data=timestamps[valid_mask])
            f_out.create_dataset('joint_positions', data=joint_positions_smooth)

            # 复制其他时序数据
            for key in f_in.keys():
                if key not in ['timestamps', 'joint_positions']:
                    if len(f_in[key].shape) > 0 and f_in[key].shape[0] == original_length:
                        # 时序数据，应用mask
                        f_out.create_dataset(key, data=f_in[key][:][valid_mask])
                    elif key.startswith('camera'):
                        # 相机数据，可能帧数不同
                        f_out.create_dataset(key, data=f_in[key][:])
                    else:
                        # 非时序数据，直接复制
                        f_out.create_dataset(key, data=f_in[key][:])

            # 复制元数据
            for attr_name in f_in.attrs.keys():
                f_out.attrs[attr_name] = f_in.attrs[attr_name]

            # 添加清洗信息
            f_out.attrs['cleaned'] = True
            f_out.attrs['original_length'] = original_length
            f_out.attrs['cleaned_length'] = clean_length
            f_out.attrs['retention_rate'] = clean_length / original_length

    print(f"\n✅ 清洗完成，数据已保存到: {output_file}")


def main():
    """主函数"""
    if len(sys.argv) < 2:
        print("用法: python3 clean_data.py <episode_file.h5> [output_file.h5]")
        print("示例: python3 clean_data.py episode_001.h5")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else None

    print("="*60)
    print("  数据清洗工具")
    print("="*60 + "\n")

    clean_episode(input_file, output_file)


if __name__ == '__main__':
    main()
