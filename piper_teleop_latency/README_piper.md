# Piper机械臂SDK使用指南

## 已完成的设置

1. ✅ 克隆了piper_sdk仓库
2. ✅ 创建了Python虚拟环境 `piper_venv`
3. ✅ 安装了依赖：python-can (4.6.1)
4. ✅ 安装了piper_sdk (版本 0.6.2)
5. ✅ 配置了CAN模块 (can0, 波特率1000000)
6. ✅ 测试验证：固件S-V1.8-9，所有功能正常

## 使用前准备

### 1. 激活虚拟环境
```bash
source piper_venv/bin/activate
```

### 2. 配置CAN模块

首先需要找到并激活CAN模块：

```bash
cd piper_sdk/piper_sdk
bash find_all_can_port.sh
```

如果检测到CAN模块，激活它：
```bash
bash can_activate.sh can0 1000000
```

注意：
- `can0` 是CAN端口名称（可以自定义）
- `1000000` 是波特率（必须是1000000）

### 3. 检查CAN模块是否激活成功
```bash
ifconfig
```
应该能看到 `can0` 接口

## 测试脚本说明

### test_piper_switch_slave.py
切换机械臂到从臂模式（需要先执行此脚本才能读取关节角度）

```bash
python test_piper_switch_slave.py
```

### test_piper_basic.py
基础测试 - 读取固件版本和关节角度

```bash
python test_piper_basic.py
```

### test_piper_tcp_pose.py
读取机械臂末端位姿（TCP位置和姿态）

```bash
python test_piper_tcp_pose.py
```

## 常见问题

1. **无法连接CAN模块**
   - 检查CAN模块是否正确连接
   - 确认已执行 `can_activate.sh` 激活CAN端口
   - 使用 `ifconfig` 确认CAN接口存在

2. **读取不到关节角度**
   - 确保机械臂处于从臂模式（运行 `test_piper_switch_slave.py`）
   - 检查机械臂是否上电

3. **SendCanMessage(SEND_MESSAGE_FAILED)**
   - CAN模块未成功连接到机械臂
   - 检查连接线
   - 将机械臂断电后重新上电

## 更多示例

查看官方demo示例：
```bash
cd piper_sdk/piper_sdk/demo/V2/
ls *.py
```

## 文档资源

- 接口详细说明：`piper_sdk/asserts/V2/INTERFACE_V2.MD`
- 中文README：`piper_sdk/README(ZH).MD`
- Q&A：`piper_sdk/asserts/Q&A.MD`
