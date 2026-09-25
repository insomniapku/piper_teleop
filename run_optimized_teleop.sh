#!/bin/bash
# Piper低延迟遥操作启动脚本

set -e

echo "======================================================================"
echo "  Piper机械臂 Pico遥操作 - 低延迟优化版启动脚本"
echo "======================================================================"
echo ""

# 切换到脚本所在目录
cd "$(dirname "$0")"

# 检查Python
if ! command -v python3 &> /dev/null; then
    echo "错误: 未找到python3"
    exit 1
fi

# 检查必要的文件
if [ ! -f "pico_teleop_optimized.py" ]; then
    echo "错误: 未找到 pico_teleop_optimized.py"
    exit 1
fi

# 显示选项菜单
echo "请选择运行模式:"
echo ""
echo "1) 安全测试模式 (速度限制: 3cm/s, 缩放: 0.5)"
echo "2) 正常模式 (速度: 8cm/s, 推荐) ⭐"
echo "3) 快速模式 (速度: 12cm/s)"
echo "4) 精细模式 (速度: 5cm/s, 适合精细操作)"
echo "5) 自定义参数"
echo "0) 退出"
echo ""
read -p "请输入选项 [0-5]: " choice

case $choice in
    1)
        echo ""
        echo "启动: 安全测试模式"
        echo "参数: --safe-test"
        echo ""
        python3 pico_teleop_optimized.py --safe-test
        ;;
    2)
        echo ""
        echo "启动: 正常模式"
        echo "参数: 默认 (50Hz, 0.08m/s)"
        echo ""
        python3 pico_teleop_optimized.py
        ;;
    3)
        echo ""
        echo "启动: 快速模式"
        echo "参数: --max-speed 0.12 --speed-percent 60"
        echo ""
        python3 pico_teleop_optimized.py --max-speed 0.12 --speed-percent 60
        ;;
    4)
        echo ""
        echo "启动: 精细模式"
        echo "参数: --max-speed 0.05 --speed-percent 30"
        echo ""
        python3 pico_teleop_optimized.py --max-speed 0.05 --speed-percent 30
        ;;
    5)
        echo ""
        echo "自定义参数模式"
        echo ""
        read -p "控制频率 (Hz) [50]: " rate
        rate=${rate:-50}

        read -p "最大速度 (m/s) [0.08]: " speed
        speed=${speed:-0.08}

        read -p "位置缩放 [1.0]: " scale
        scale=${scale:-1.0}

        read -p "速度百分比 (1-100) [40]: " percent
        percent=${percent:-40}

        echo ""
        echo "启动: 自定义模式"
        echo "参数: --control-rate $rate --max-speed $speed --position-scale $scale --speed-percent $percent"
        echo ""
        python3 pico_teleop_optimized.py \
            --control-rate $rate \
            --max-speed $speed \
            --position-scale $scale \
            --speed-percent $percent
        ;;
    0)
        echo "退出"
        exit 0
        ;;
    *)
        echo "无效选项"
        exit 1
        ;;
esac

echo ""
echo "======================================================================"
echo "  程序已退出"
echo "======================================================================"
