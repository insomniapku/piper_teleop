# Piper Robotic Arm Control & Teleoperation

这是 Piper + Pico XR 遥操作仓库。V1 Cartesian Stable 是冻结回退版本；
V2 Joint IK 已完成离线测试和操作者在场真机测试，仍处于现场调参阶段。

## 支持状态

| 版本 | 状态 | 入口 |
|---|---|---|
| V1 Cartesian Stable | **SUPPORTED**，当前正式版 | `./run_piper_normal_teleop.sh` |
| V1 Safe Test | **SUPPORTED**，首次连接/恢复测试 | `./run_piper_safe_teleop.sh` |
| V2 Joint IK | **EXPERIMENTAL / LIVE TESTED**，现场调参 | `pico_teleop_piper_ik_v2.py` |
| 旧 Python/ROS/IK 实验 | **LEGACY** | 不作为真机入口 |

详细版本、完整操作流程和故障排查：

- [VERSIONS.md](VERSIONS.md)
- [docs/OPERATIONS_ZH.md](docs/OPERATIONS_ZH.md)

## 当前正式版 V1

工作站上的正式启动方式：

```bash
cd /home/zktitan/piper_teleop_latency
./run_piper_normal_teleop.sh
```

启动脚本固定使用已经过真机验证的参数：

```text
controller-hand       = left
position-scale        = 0.8
speed-percent         = 100
Python speed limit    = disabled
Python workspace clip = disabled
gripper               = binary, fully open/fully closed
orientation           = hold on clutch engage
CAN                    = can0
XR-to-robot yaw        = 0 deg
```

操作：

- 按住左手 Grip：机械臂位置跟随；
- 松开 Grip：暂停跟随，可把手柄重新放回舒适位置；
- 左手 Trigger：完整张开或完整闭合夹爪；
- 先松开 Grip，再按 `Ctrl-C` 停止程序。

程序显示安全提示时，确认机械臂周围无人和障碍物后再输入 `ARM`。

## 首次连接或恢复后的 Safe Test

不要在网络/CAN 恢复后直接运行正式速度。先执行：

```bash
cd /home/zktitan/piper_teleop_latency
./run_piper_safe_teleop.sh
```

Safe Test 使用 20% 硬件速度、小范围位移且不控制夹爪。依次确认前后、左右、
上下方向后，再停止 Safe Test 并启动正式版。

## V2 Joint IK 现场测试入口

V2 使用四元数/旋转矩阵保持末端姿态，Pinocchio 求解 J1-J6，并通过
`JointCtrl` 发送连续关节目标。当前工作站使用 V1 的共享虚拟环境：

```bash
cd /home/zktitan/piper_teleop_ik_v2
/home/zktitan/piper_teleop_latency/.venv/bin/python -u \
  pico_teleop_piper_ik_v2.py \
  --hardware \
  --controller-hand left \
  --position-scale 0.8 \
  --rotation-scale 1.0 \
  --max-orientation-delta-deg 180 \
  --max-joint-step-deg 2.0 \
  --no-speed-limit \
  --no-workspace-limit \
  --speed-percent 100 \
  --binary-gripper \
  --can-name can0 \
  --yaw-deg 0
```

正式现场参数保留 `--max-joint-step-deg 2.0`，在 50 Hz 下限制每个关节每帧最多
变化 2°。不要在真机正式操作中使用 `--no-joint-step-limit`；完全取消该保护曾导致
raw IK 大跳步和机械臂接近硬限位。首次连接或修改 IK 后仍必须先运行 `--safe-test`。

## Pico 连接

Linux 上先启动 XRoboToolkit PC Service，再打开 Pico 的 XRoboToolkit 应用。
Pico 输入的端口必须以 PC Service **当前显示**为准，不要固定使用历史端口。

```bash
/opt/apps/roboticsservice/runService.sh
```

如果 PC Service 已通过桌面图标运行，不要再启动第二个实例。

## 冻结版本

```text
branch: snapshot/piper-teleop-v1-working-20260826
tag:    piper-teleop-v1-working-20260826
commit: 6b2b18b08d394118fcfdc473df25d957c9fe27b5
```

`pico_teleop_piper.py`、`pico_teleop_improved.py` 和旧 PyKDL/Placo 文件只保留作
历史参考。不要在正式遥操作运行时并行启动 SDK 测试脚本或第二个 CAN 控制进程。

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
# 启动脚本固定使用仓库根目录的 .venv
# --system-site-packages 允许复用 Ubuntu/ROS 已安装的 CAN、SciPy 和 XR 绑定
python3 -m venv --system-site-packages .venv
source .venv/bin/activate

# 安装仓库内 Piper SDK 和遥操作包
python -m pip install -e ./piper_sdk
python -m pip install -e .

# 验证正式入口需要的模块
python -c "import xrobotoolkit_sdk, piper_sdk, can, numpy, scipy; print('runtime imports: OK')"
```

`xrobotoolkit_sdk` 来自 XRoboToolkit PC Service Python binding。若最后的导入检查
失败，请先按照 XRoboToolkit 官方说明安装/编译 PC Service Pybind；不要从不明来源
安装同名包。当前工作站验证环境为 Python 3.12、`xrobotoolkit-sdk 1.0.2`、
`piper-sdk 0.6.2`、`python-can 4.3.1`、NumPy 1.26.4、SciPy 1.11.4。

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
