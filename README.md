# Piper Robotic Arm Control & Teleoperation

这是一个用于Piper机械臂控制和遥操作的完整项目，包括SDK控制、ROS集成和遥操作功能。

## 项目结构

```
lcz0820/
├── piper_sdk/              # Piper SDK (官方)
├── piper_teleop/           # 遥操作包
├── piper_ros/              # ROS集成包
├── test_piper_*.py         # SDK测试脚本
├── PIPER_QUICK_REFERENCE.md # API快速参考
└── README.md               # 本文件
```

## 硬件要求

- Piper机械臂（固件版本：S-V1.8-9 或更高）
- USB转CAN模块（官方或兼容模块）
- Ubuntu 18.04/20.04/22.04

## 软件依赖

- Python 3.6+
- python-can >= 3.3.4
- ROS Noetic (可选，用于ROS功能)

## 快速开始

### 1. 安装依赖

```bash
# 创建虚拟环境
python3 -m venv piper_venv
source piper_venv/bin/activate

# 安装python-can
pip install python-can

# 安装piper_sdk
cd piper_sdk
pip install .
cd ..
```

### 2. 配置CAN模块

```bash
# 安装CAN工具
sudo apt update && sudo apt install can-utils ethtool

# 查找CAN模块
cd piper_sdk/piper_sdk
bash find_all_can_port.sh

# 激活CAN模块
# 如果只有一个CAN模块：
bash can_activate.sh can0 1000000

# 如果有多个CAN模块，需要指定USB端口：
bash can_activate.sh can0 1000000 "1-11.2:1.0"

# 验证CAN接口是否激活
ifconfig | grep can0
```

### 3. 运行测试

```bash
# 回到项目根目录
cd ~/lcz0820

# 激活虚拟环境
source piper_venv/bin/activate

# 基础测试 - 读取固件版本和关节角度
python test_piper_basic.py

# 夹爪控制测试
python test_piper_gripper.py

# 关节运动测试
python test_piper_joint_control.py

# 笛卡尔空间运动测试
python test_piper_cartesian.py

# 完整功能演示（推荐）
python test_piper_complete_demo.py
```

## 测试脚本说明

| 脚本 | 功能描述 |
|------|----------|
| `test_piper_basic.py` | 读取固件版本和关节角度 |
| `test_piper_switch_slave.py` | 切换机械臂到从臂模式 |
| `test_piper_tcp_pose.py` | 实时显示末端位姿 (TCP) |
| `test_piper_gripper.py` | 夹爪控制演示（开/关/半开） |
| `test_piper_joint_control.py` | 关节空间运动控制 |
| `test_piper_cartesian.py` | 笛卡尔空间直线运动 |
| `test_piper_complete_demo.py` | **完整功能演示（推荐从这里开始）** |

## 测试结果

✅ **已验证功能**：
- 固件通信：正常，200Hz
- 夹爪控制：精度±0.5mm
- 关节运动：6轴控制正常
- 笛卡尔运动：MOVEP和MOVEL模式正常
- 状态读取：关节角度、末端位姿、夹爪状态

## 核心API使用示例

### 初始化

```python
from piper_sdk import *
import time

# 创建接口
piper = C_PiperInterface_V2(can_name="can0", judge_flag=False)
piper.ConnectPort()
time.sleep(0.1)

# 使能机械臂
while not piper.EnablePiper():
    time.sleep(0.01)
```

### 关节控制

```python
# 设置运动模式
piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)  # 使能，位置模式，50%速度

# 关节角度（单位：毫度）
factor = 57295.7795  # 弧度转毫度
position_rad = [0.2, 0.2, -0.2, 0.3, -0.2, 0.5]
joints = [round(p * factor) for p in position_rad]
piper.JointCtrl(*joints)
```

### 笛卡尔控制

```python
# MOVEL模式 - 直线运动
piper.MotionCtrl_2(0x01, 0x02, 50, 0x00)

# 位置单位：微米，姿态单位：毫度
piper.EndPoseCtrl(150000, -50000, 150000, -179900, 0, -179900)
```

### 夹爪控制

```python
# 回零
piper.GripperCtrl(0, 1000, 0x02, 0)
time.sleep(1)

# 使能
piper.GripperCtrl(0, 1000, 0x01, 0)
time.sleep(1)

# 打开到50mm（单位：微米）
piper.GripperCtrl(50000, 1000, 0x01, 0)
```

更多API详情请参考：[PIPER_QUICK_REFERENCE.md](PIPER_QUICK_REFERENCE.md)

## 遥操作功能

### 配置

```bash
# 进入遥操作包
cd piper_teleop

# 配置参数（编辑config/config.yaml）
# 设置主从臂的CAN端口、控制模式等
```

### 运行遥操作

```bash
# 启动遥操作节点
# （具体命令取决于你的遥操作实现）
roslaunch piper_teleop teleop.launch
```

详细文档请参考：[PIPER_TELEOP_ISSUES.md](PIPER_TELEOP_ISSUES.md)

## 单位转换参考

| 参数 | SDK单位 | 实际单位 | 转换 |
|------|---------|----------|------|
| 位置 (X,Y,Z) | 微米 (μm) | 毫米 (mm) | ÷1000 |
| 角度 | 毫度 | 度 (°) | ÷1000 |
| 弧度 → 毫度 | - | - | ×57295.7795 |
| 夹爪位置 | 微米 (μm) | 毫米 (mm) | ÷1000 |

## 工作空间范围

- **X轴**: 约 -300mm ~ 300mm
- **Y轴**: 约 -300mm ~ 300mm  
- **Z轴**: 约 100mm ~ 400mm
- **夹爪**: 0mm ~ 70mm

## 故障排除

### CAN模块未检测到

```bash
# 检查USB连接
lsusb

# 检查模块是否被识别
bash find_all_can_port.sh

# 重新插拔CAN模块
```

### 机械臂无法使能

1. 确认机械臂已上电
2. 检查CAN连接是否正常
3. 确认处于从臂模式（运行 `test_piper_switch_slave.py`）
4. 检查CAN接口激活状态：`ifconfig | grep can0`

### 运动指令无响应

1. 确认已调用 `MotionCtrl_2` 设置运动模式
2. 检查目标位置是否在工作空间内
3. 查看机械臂状态：`piper.GetArmStatus()`
4. 降低运动速度重试

### SendCanMessage(SEND_MESSAGE_FAILED)

1. 机械臂断电后重新上电
2. 检查CAN模块与机械臂的连接
3. 重新激活CAN接口

## 安全注意事项

⚠️ **使用前请务必阅读**：

1. **运行脚本前**确保机械臂周围没有障碍物和人员
2. **初次测试**建议使用较低速度（30-50%）
3. **随时准备**按下急停按钮
4. **确保目标位置**在机械臂工作空间内
5. **避免突然**的大幅度运动

## 项目文档

- [API快速参考](PIPER_QUICK_REFERENCE.md) - 核心API和代码示例
- [遥操作问题](PIPER_TELEOP_ISSUES.md) - 遥操作功能的已知问题
- [解决方案总结](SOLUTION_SUMMARY.md) - 技术方案和优化
- [数据采集指南](README_DATA_COLLECTION.md) - 数据采集流程

## 参考资源

- Piper SDK官方仓库: https://github.com/agilexrobotics/piper_sdk
- Piper SDK文档: `piper_sdk/README(ZH).MD`
- 官方Demo: `piper_sdk/piper_sdk/demo/V2/`
- Discord社区: https://discord.gg/wrKYTxwDBd

## 许可证

本项目基于MIT许可证发布。Piper SDK遵循其自身的许可证。

## 贡献

欢迎提交Issue和Pull Request！

## 联系方式

- GitHub: [@insomniapku](https://github.com/insomniapku)

---

最后更新：2026-08-25
