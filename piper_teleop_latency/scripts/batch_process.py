#!/usr/bin/env python3
"""
批量处理工具
批量清洗和验证采集的数据
"""

import os
import sys
import subprocess
from pathlib import Path


def batch_process(data_dir):
    """批量处理所有episode"""

    print("="*70)
    print("  批量数据处理工具")
    print("="*70 + "\n")

    # 查找所有原始episode
    episodes = sorted([f for f in os.listdir(data_dir)
                      if f.startswith('episode_') and f.endswith('.h5') and '_clean' not in f])

    if not episodes:
        print(f"❌ 在 {data_dir} 中未找到episode文件")
        sys.exit(1)

    print(f"📁 数据目录: {data_dir}")
    print(f"📊 找到 {len(episodes)} 个episode")
    print()

    # 创建processed目录
    processed_dir = os.path.join(data_dir, 'processed')
    os.makedirs(processed_dir, exist_ok=True)

    clean_count = 0
    failed_count = 0

    for i, episode in enumerate(episodes, 1):
        print(f"[{i}/{len(episodes)}] 处理 {episode}...")

        input_file = os.path.join(data_dir, episode)
        output_file = os.path.join(processed_dir, episode.replace('.h5', '_clean.h5'))

        try:
            # 调用清洗脚本
            result = subprocess.run(
                ['python3', 'clean_data.py', input_file, output_file],
                capture_output=True,
                text=True,
                timeout=30
            )

            if result.returncode == 0:
                clean_count += 1
                print(f"  ✅ 成功")
            else:
                failed_count += 1
                print(f"  ❌ 失败: {result.stderr[:100]}")

        except Exception as e:
            failed_count += 1
            print(f"  ❌ 错误: {e}")

        print()

    # 总结
    print("="*70)
    print("  处理完成")
    print("="*70)
    print(f"✅ 成功: {clean_count}")
    print(f"❌ 失败: {failed_count}")
    print(f"📁 清洗后数据保存在: {processed_dir}")
    print("="*70 + "\n")


def split_dataset(data_dir, train_ratio=0.8):
    """划分训练集和测试集"""

    print("="*70)
    print("  划分数据集")
    print("="*70 + "\n")

    processed_dir = os.path.join(data_dir, 'processed')

    if not os.path.exists(processed_dir):
        print(f"❌ 目录不存在: {processed_dir}")
        print(f"请先运行批量清洗")
        sys.exit(1)

    # 获取所有清洗后的episode
    episodes = sorted([f for f in os.listdir(processed_dir) if f.endswith('_clean.h5')])

    if not episodes:
        print(f"❌ 未找到清洗后的数据")
        sys.exit(1)

    print(f"📊 找到 {len(episodes)} 个清洗后的episode")

    # 随机打乱
    import random
    random.shuffle(episodes)

    # 划分
    split_idx = int(len(episodes) * train_ratio)
    train_episodes = episodes[:split_idx]
    test_episodes = episodes[split_idx:]

    print(f"📊 训练集: {len(train_episodes)} episodes")
    print(f"📊 测试集: {len(test_episodes)} episodes")

    # 创建目录
    train_dir = os.path.join(data_dir, 'train')
    test_dir = os.path.join(data_dir, 'test')
    os.makedirs(train_dir, exist_ok=True)
    os.makedirs(test_dir, exist_ok=True)

    # 创建符号链接或复制文件
    import shutil

    print(f"\n复制文件...")
    for ep in train_episodes:
        src = os.path.join(processed_dir, ep)
        dst = os.path.join(train_dir, ep)
        shutil.copy2(src, dst)

    for ep in test_episodes:
        src = os.path.join(processed_dir, ep)
        dst = os.path.join(test_dir, ep)
        shutil.copy2(src, dst)

    print(f"\n✅ 数据集划分完成")
    print(f"📁 训练集: {train_dir}")
    print(f"📁 测试集: {test_dir}")
    print("="*70 + "\n")


def main():
    """主函数"""
    if len(sys.argv) < 2:
        print("用法:")
        print("  python3 batch_process.py <data_dir> clean    # 批量清洗")
        print("  python3 batch_process.py <data_dir> split    # 划分数据集")
        print("\n示例:")
        print("  python3 batch_process.py $HOME/data/piper_demos clean")
        sys.exit(1)

    data_dir = sys.argv[1]
    command = sys.argv[2] if len(sys.argv) > 2 else 'clean'

    if not os.path.exists(data_dir):
        print(f"❌ 目录不存在: {data_dir}")
        sys.exit(1)

    if command == 'clean':
        batch_process(data_dir)
    elif command == 'split':
        split_dataset(data_dir)
    else:
        print(f"❌ 未知命令: {command}")
        print(f"支持的命令: clean, split")
        sys.exit(1)


if __name__ == '__main__':
    main()
