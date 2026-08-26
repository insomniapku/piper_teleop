#!/bin/bash
# Pico头显应用安装脚本

echo "======================================================================"
echo "  XRoboToolkit Pico应用安装程序"
echo "======================================================================"
echo ""

APK_PATH="/home/zktitan/Downloads/XRoboToolkit-PICO-1.1.1.apk"

# 检查APK文件
if [ ! -f "$APK_PATH" ]; then
    echo "✗ 错误: APK文件不存在"
    echo "  路径: $APK_PATH"
    exit 1
fi

echo "✓ 找到APK文件: $APK_PATH"
echo ""

# 检查ADB
if ! command -v adb &> /dev/null; then
    echo "✗ 错误: ADB未安装"
    echo ""
    echo "安装方法:"
    echo "  sudo apt-get install android-tools-adb android-tools-fastboot"
    exit 1
fi

echo "✓ ADB已安装"
echo ""

# 检查设备连接
echo "检查Pico设备连接..."
DEVICES=$(adb devices | grep -v "List" | grep "device$" | wc -l)

if [ "$DEVICES" -eq 0 ]; then
    echo ""
    echo "⚠ 未检测到Pico设备"
    echo ""
    echo "请按以下步骤操作:"
    echo "  1. 在Pico头显中启用开发者模式:"
    echo "     设置 → 系统 → 关于 → 多次点击'版本号'"
    echo ""
    echo "  2. 启用USB调试:"
    echo "     设置 → 系统 → 开发者选项 → USB调试"
    echo ""
    echo "  3. 用USB-C线连接Pico到电脑"
    echo ""
    echo "  4. 在头显中确认USB调试授权弹窗"
    echo ""
    echo "  5. 重新运行此脚本"
    exit 1
fi

echo "✓ 检测到 $DEVICES 个设备"
echo ""
adb devices
echo ""

# 检查是否已安装
PACKAGE="com.pico.xrobotoolkit"
echo "检查是否已安装旧版本..."
if adb shell pm list packages | grep -q "$PACKAGE"; then
    echo "⚠ 检测到已安装的版本，正在卸载..."
    adb uninstall "$PACKAGE"
    echo "✓ 旧版本已卸载"
    echo ""
fi

# 安装APK
echo "正在安装 XRoboToolkit 到 Pico..."
adb install "$APK_PATH"

if [ $? -eq 0 ]; then
    echo ""
    echo "======================================================================"
    echo "  ✓ 安装成功！"
    echo "======================================================================"
    echo ""
    echo "下一步操作:"
    echo "  1. 戴上Pico头显"
    echo "  2. 打开应用库，查找'XRoboToolkit'"
    echo "  3. 启动应用"
    echo "  4. 在应用中配置:"
    echo "     - Server IP: 192.168.111.123"
    echo "     - Port: 60061"
    echo "  5. 点击'Connect'连接"
    echo ""
    echo "然后在PC上运行测试:"
    echo "  cd /home/zktitan/lcz0820"
    echo "  python3 test_sdk_basic.py"
    echo ""
else
    echo ""
    echo "✗ 安装失败"
    echo ""
    echo "常见问题:"
    echo "  1. USB调试未授权 - 在头显中确认弹窗"
    echo "  2. 开发者模式未启用 - 检查设置"
    echo "  3. USB连接不稳定 - 重新插拔USB"
    exit 1
fi
