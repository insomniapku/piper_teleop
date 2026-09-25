# XRoboToolkit 控制器数据为0 - 故障排除指南

## 问题描述

从测试输出可以看到：
```
initialize sdk,connect127.0.0.1:60061
client start server stream server connect
device found: TestDevice
```

但是控制器数据全为0：
```
位置=[0.000, 0.000, 0.000]
握把=0.00, 扳机=0.00
```

## 关键发现

1. ✓ SDK成功连接到PC Service (127.0.0.1:60061)
2. ✓ 发现了设备 (device found: TestDevice)
3. ✗ 但是数据全为0

**重要**: "TestDevice"可能表示这是一个测试/模拟设备，而不是真实的Pico头显。

---

## 解决方案

### 方案1: 检查PC Service中的设备连接 ⭐ 推荐

1. **打开XRoboToolkit PC Service程序**
   - 查看主界面的设备列表
   - 确认是否显示Pico头显已连接

2. **检查设备名称**
   - 如果显示"TestDevice" → 这是模拟设备，需要连接真实设备
   - 如果显示"Pico 4" 或类似名称 → 真实设备已连接

3. **重新扫描设备**
   - 在PC Service中点击"刷新"或"重新扫描"按钮
   - 确保Pico头显已开机并通过USB连接

---

### 方案2: 配置Pico头显WiFi连接

你提到头显IP是`192.168.111.85`，可以尝试WiFi连接：

1. **在PC Service中添加设备**
   - 打开设置/配置界面
   - 查找"添加设备"或"网络设备"选项
   - 输入IP: `192.168.111.85`

2. **确认网络连接**
   ```bash
   # 测试是否能ping通
   ping 192.168.111.85
   
   # 检查网络接口
   ifconfig
   ```

3. **检查防火墙**
   - 确保防火墙不阻止PC Service与头显通信
   - 可能需要开放特定端口

---

### 方案3: 使用USB连接（推荐）

USB连接通常更稳定：

1. **检查USB连接**
   ```bash
   # 查看是否识别到Pico设备
   lsusb | grep -i "pico\|qualcomm\|android"
   
   # 查看ADB设备（如果Pico使用ADB协议）
   adb devices
   ```

2. **启用USB调试**
   - 戴上Pico头显
   - 进入设置 → 开发者选项
   - 启用"USB调试"
   - 连接USB后确认授权提示

3. **切换USB模式**
   - 某些设备需要切换到"文件传输(MTP)"或"PTP"模式
   - 尝试不同的USB接口

---

### 方案4: 检查PC Service配置

1. **查看PC Service日志**
   - 在程序界面查找"日志"或"Log"标签
   - 检查是否有错误信息

2. **重启PC Service**
   ```bash
   # 完全关闭PC Service
   pkill -f XRobo
   
   # 重新启动PC Service程序
   ```

3. **检查版本兼容性**
   - SDK版本: 1.0.2
   - 确保PC Service版本与SDK匹配
   - 必要时更新到最新版本

---

### 方案5: 使用诊断脚本

运行详细诊断：

```bash
python3 diagnose_xr_connection.py
```

这个脚本会：
- 检查所有数据源（控制器、头显、按钮）
- 测试手部追踪
- 检查时间戳
- 提供针对性的诊断建议

---

## 可能的原因分析

### 原因1: TestDevice问题 ⭐ 最可能

"TestDevice"通常表示：
- PC Service处于测试模式
- 没有找到真实设备
- 使用模拟设备运行

**解决**: 确保真实Pico设备已连接并被PC Service识别

---

### 原因2: 设备未授权

XRoboToolkit需要设备授权才能读取数据。

**检查步骤**:
1. 连接Pico后，头显内应显示授权提示
2. 确认授权后，PC Service才能读取数据
3. 如果没有提示，重新插拔USB

---

### 原因3: 通信协议问题

PC Service与Pico的通信可能有问题。

**检查**:
```bash
# 查看PC Service进程
ps aux | grep XRobo

# 查看网络连接
netstat -anp | grep 60061
```

---

### 原因4: 头显未启动应用

某些情况下需要在Pico头显上运行配套应用。

**检查**:
- 在Pico头显的应用列表中查找XRoboToolkit相关应用
- 如果有，启动该应用
- 保持应用在前台运行

---

## 调试步骤清单

按顺序执行：

- [ ] **步骤1**: 确认XRoboToolkit PC Service正在运行
  ```bash
  ps aux | grep XRobo
  ```

- [ ] **步骤2**: 打开PC Service程序界面，查看设备列表
  - 是否显示"TestDevice"？
  - 是否显示真实Pico设备？

- [ ] **步骤3**: 检查USB连接
  ```bash
  lsusb | grep -i "pico\|qualcomm"
  ```

- [ ] **步骤4**: 在Pico头显上检查
  - 开发者选项是否启用？
  - USB调试是否启用？
  - 是否看到授权提示？

- [ ] **步骤5**: 运行诊断脚本
  ```bash
  python3 diagnose_xr_connection.py
  ```

- [ ] **步骤6**: 尝试WiFi连接（如果USB不行）
  - 在PC Service中添加IP: 192.168.111.85
  - 确认网络可达: `ping 192.168.111.85`

- [ ] **步骤7**: 重启所有组件
  - 关闭PC Service
  - 重启Pico头显
  - 重新连接USB/WiFi
  - 重新启动PC Service

---

## XRoboToolkit PC Service 使用提示

### 寻找PC Service程序

**Windows**:
- 通常安装在: `C:\Program Files\XRoboToolkit\`
- 可执行文件: `XRoboToolkit PC Service.exe`

**Linux**:
- 需要从源码编译
- 参考: https://github.com/XR-Robotics/XRoboToolkit-PC-Service

### PC Service界面应该显示什么

正常情况下应该看到：
```
设备列表:
  ✓ Pico 4 [已连接]
    - 状态: 正常
    - 连接方式: USB / WiFi
    - 控制器: 左右手已连接
```

如果看到：
```
设备列表:
  ? TestDevice [模拟]
```
说明没有连接真实设备。

---

## 如果以上方法都不行

### 最后的尝试

1. **完全卸载重装PC Service**
   - 卸载现有版本
   - 下载最新版本
   - 重新安装

2. **尝试在另一台电脑上测试**
   - 排除是否是系统环境问题

3. **联系技术支持**
   - XRoboToolkit GitHub: https://github.com/XR-Robotics
   - 提供PC Service日志
   - 说明Pico型号和系统版本

4. **使用备用方案**
   - 如果Pico连接一直有问题
   - 考虑使用其他VR设备（如HTC Vive, Oculus）
   - 或者使用ROS直接控制（不通过VR）

---

## 补充: 头显IP的作用

你提到头显IP是`192.168.111.85`，这通常用于：

1. **WiFi流媒体模式**
   - PC Service通过网络传输数据
   - 无需USB连接
   - 需要在同一局域网

2. **ADB over Network**
   - 通过网络进行调试
   - 需要先通过USB配对

3. **配置方法**
   ```bash
   # 如果支持ADB
   adb connect 192.168.111.85:5555
   
   # 在PC Service中
   # 手动添加设备 → 输入IP → 连接
   ```

---

## 相关命令速查

```bash
# 查看PC Service进程
ps aux | grep XRobo

# 查看USB设备
lsusb

# 测试网络连接
ping 192.168.111.85

# 运行诊断
python3 diagnose_xr_connection.py

# 运行测试
python3 test_sdk_basic.py

# 查看SDK版本
pip show xrobotoolkit-sdk
```

---

## 成功标志

当问题解决后，你应该看到：

```bash
python3 diagnose_xr_connection.py

右手控制器位置: [0.123, -0.045, 0.678]  # 非零值
头显位置: [0.050, 1.650, -0.120]         # 非零值
握把: 1.00, 扳机: 0.50                    # 响应输入
```

然后就可以正常运行遥操作程序了！

---

**文档创建**: 2026-08-26  
**针对问题**: 控制器数据全为0，显示TestDevice
