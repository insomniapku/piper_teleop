# Pico数据采集系统 - 快速入门

## ✅ 系统已就绪

所有组件已安装并可用！

---

## 🚀 启动方式

### 方法1: 使用Bash脚本（推荐）

```bash
cd /home/zktitan/lcz0820/piper_teleop
chmod +x scripts/quick_start.sh
./scripts/quick_start.sh
```

### 方法2: 使用ROS 2 Launch

```bash
cd /home/zktitan/lcz0820
source install/setup.bash

# Mock模式（无需真实硬件）
ros2 launch piper_teleop data_collection.launch.py \
  xr_mode:=mock \
  record_camera:=false
```

### 方法3: 单独启动节点

```bash
cd /home/zktitan/lcz0820
source install/setup.bash

# 启动数据采集节点
ros2 run piper_teleop data_collection_node
```

---

## 📊 可用的工具

### 1. 测试XR接口
```bash
source install/setup.bash
ros2 run piper_teleop xr_test_node --ros-args -p mode:=mock
```

### 2. 数据可视化
```bash
python3 /home/zktitan/lcz0820/piper_teleop/scripts/visualize_data.py \
  /home/zktitan/data/piper_demos/episode_001.h5
```

### 3. 数据清洗
```bash
python3 /home/zktitan/lcz0820/piper_teleop/scripts/clean_data.py \
  /home/zktitan/data/piper_demos/episode_001.h5
```

### 4. 批量处理
```bash
# 批量清洗
python3 /home/zktitan/lcz0820/piper_teleop/scripts/batch_process.py \
  /home/zktitan/data/piper_demos clean

# 划分数据集
python3 /home/zktitan/lcz0820/piper_teleop/scripts/batch_process.py \
  /home/zktitan/data/piper_demos split
```

---

## 📁 已安装的文件

### 核心节点
- ✅ `data_collection_node` - 数据采集
- ✅ `xr_test_node` - XR接口测试
- ✅ `xr_monitor` - XR监控
- ✅ `piper_teleop_node` - 遥操作控制

### Launch文件
- ✅ `data_collection.launch.py` - 完整数据采集系统
- ✅ `teleop.launch.py` - 基础遥操作

### 工具脚本
- ✅ `quick_start.sh` - 快速启动
- ✅ `clean_data.py` - 数据清洗
- ✅ `visualize_data.py` - 数据可视化
- ✅ `batch_process.py` - 批量处理

### 配置文件
- ✅ `teleop_config.yaml` - 遥操作配置（已修复workspace limit）

---

## 📖 完整文档

1. **PICO_DATA_COLLECTION_GUIDE.md** - 完整实施指南
2. **PIPER_TELEOP_ISSUES.md** - 问题诊断
3. **PLACO_OPTIMIZATION_SUMMARY.md** - Placo优化
4. **README_DATA_COLLECTION.md** - 项目总览

---

## 🔧 当前配置

- **XR模式**: Mock（测试模式）
- **控制频率**: 50 Hz
- **数据目录**: `/home/zktitan/data/piper_demos`
- **相机录制**: 关闭（可在launch时启用）
- **IK求解器**: PyKDL（已优化）
- **Workspace limit**: 已修复

---

## ⚡ 测试系统

运行完整测试：

```bash
cd /home/zktitan/lcz0820
source install/setup.bash

# 测试XR接口
ros2 run piper_teleop xr_test_node --ros-args -p mode:=mock

# 查看XR数据
ros2 topic echo /xr/right_controller/pose
```

---

## 🎯 下一步

### 使用Mock模式测试
```bash
./scripts/quick_start.sh
```

### 连接真实Pico（需要硬件）
```bash
ros2 launch piper_teleop data_collection.launch.py \
  xr_mode:=sdk \
  record_camera:=true
```

---

## 💡 提示

- 所有脚本都在 `/home/zktitan/lcz0820/piper_teleop/scripts/`
- 数据保存在 `/home/zktitan/data/piper_demos/`
- 使用 Mock 模式无需真实硬件即可测试
- 完整文档在项目根目录

---

**系统状态**: ✅ 已构建，可运行

**构建命令**:
```bash
cd /home/zktitan/lcz0820
colcon build --packages-select piper_teleop --symlink-install
source install/setup.bash
```
