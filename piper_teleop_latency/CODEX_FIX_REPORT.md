# Pico XR 遥操作 AgileX Piper 延迟与映射问题修复报告

## 1. 摘要

本次快速修复针对三类现场问题：

1. 手柄与机械臂运动方向或比例不一致；
2. 跟随延迟明显；
3. 手柄停止后机械臂继续执行积累目标，出现“追赶”。

修复采用独立新增入口 `pico_teleop_piper_fixed.py`，没有重写 ROS2、Placo 或 PyKDL，没有删除或覆盖旧脚本。控制链路调整为：

```text
最新 Pico position
→ 官方 XR 坐标转换
→ grip 相对位移
→ 工作空间裁剪
→ 基于真实 dt 的目标限速
→ Piper MOVEP + 最新 EndPoseCtrl
```

本次仅执行静态检查和内置无硬件 dry-run。没有初始化 XR SDK、连接 Pico、连接 `can0`，也没有发送任何真实机器人或夹爪命令。

## 2. 仓库与版本

| 项目 | 内容 |
| --- | --- |
| 实际修改仓库 | `https://github.com/gxccc123/piper_teleop.git` |
| 克隆结果 | PRIMARY HTTPS 成功，未使用 SSH 或备用仓库 |
| 修改前 commit | `69ee69349a4962652d1711769f1a0f8f4a6c813a` |
| 修复分支 | `fix/pico-teleop-latency` |
| 控制代码 commit | `043b32740fb70110f9367a722cae9596494dcaba` |
| XR 官方仓库 | `https://github.com/XR-Robotics/XRoboToolkit-Teleop-Sample-Python.git` |
| XR 官方参考 commit | `79e5cb8a56e3455515ce1b476e993c764ec58739` |

PRIMARY 仓库可正常访问，因此未使用 `insomniapku/piper_teleop`。未执行 `git push`。XR 官方仓库只读且工作树干净。

## 3. 修改范围

只修改目标仓库中的：

- `pico_teleop_piper_fixed.py`：新增低延迟、安全门控直连入口；
- `CODEX_FIX_REPORT.md`：本报告。

以下内容均未修改或删除：

- `pico_teleop_piper.py`；
- `pico_teleop_improved.py`；
- ROS2 节点、launch 和配置；
- Placo、PyKDL、IK 实现；
- Piper SDK 及其示例；
- XR 官方仓库。

没有安装或升级依赖，没有增加 pytest/CI、GUI、可视化、多线程队列或新框架。

## 4. 实际入口定位

### 4.1 结论

当前直接读取 Pico 并调用 Piper SDK 的实际入口是根目录 `pico_teleop_piper.py`。依据：

- 脚本自身声明 `python3 pico_teleop_piper.py` 为直接运行方式；
- 直接调用 `get_right_controller_pose`、`get_right_grip`、`get_right_trigger`；
- 直接构造 `C_PiperInterface_V2`；
- 直接调用 `MotionCtrl_2`、`EndPoseCtrl`、`GripperCtrl`。

`pico_teleop_improved.py` 是另一套双手、绝对位置实验入口。根 README 主要指向 ROS launch，包内 README 指向 ROS2 launch，但包内 README 仍将 Pico 映射和 Piper 硬件接口标为待实现，因此本次没有改 ROS2 路径。

### 4.2 重复文件

仓库内存在内容相同的重复 package 文件，例如：

```text
piper_teleop/xr_interface.py
piper_teleop/piper_teleop/xr_interface.py

piper_teleop/full_teleop_node.py
piper_teleop/piper_teleop/full_teleop_node.py
```

这是既有仓库结构问题。本次为最小修复，没有清理目录或迁移入口。

## 5. 官方与 SDK 对照

### 5.1 XR 官方实现

官方 `xrobotoolkit_teleop/utils/geometry.py` 定义：

```python
R_HEADSET_TO_WORLD = np.array([
    [ 0,  0, -1],
    [-1,  0,  0],
    [ 0,  1,  0],
])
```

官方 `BaseTeleopController` 先转换控制器位置，再保存 grip 激活时的控制器和末端参考，以相对位移生成目标。新脚本沿用这两个关键语义。

### 5.2 Piper SDK

仓库内 SDK 示例和接口定义确认：

- `MotionCtrl_2(0x01, 0x00, speed, 0x00)`：MOVEP；
- `MotionCtrl_2(0x01, 0x02, speed, 0x00)`：MOVEL；
- `EndPoseCtrl` 位置单位：`0.001 mm`；
- 1 米对应整数 `1,000,000`；
- 末端反馈位置也是 `0.001 mm`；
- RX/RY/RZ 反馈单位为 `0.001°`。

新脚本继续使用这些单位，未引入新的转换层。

## 6. 根因与修改

### 6.1 坐标系未转换

旧代码直接计算：

```python
delta = (xr_pos - xr_ref_pos) * position_scale
```

这错误假设 XR 与 Piper 的三轴同向。官方默认关系实际为：

```text
Piper X = -XR Z
Piper Y = -XR X
Piper Z =  XR Y
```

新脚本集中定义：

```python
R_XR_TO_PIPER = np.array([
    [ 0.0,  0.0, -1.0],
    [-1.0,  0.0,  0.0],
    [ 0.0,  1.0,  0.0],
], dtype=np.float64)
```

并计算：

```python
R_FINAL = R_INSTALLATION @ R_XR_TO_PIPER
xr_pos_robot = R_FINAL @ xr_pos
delta_robot = xr_pos_robot - xr_ref_pos_robot
target = ee_ref_pos + position_scale * delta_robot
```

`--yaw-deg` 只允许 0、90、-90、180，解决常见安装朝向差异，不做复杂自动标定。

### 6.2 比例过大

旧直连入口默认 `position_scale=3.5`，会将尚未正确转换的输入进一步放大。

新脚本：

- 正常默认：`1.0`；
- `--safe-test`：强制 `0.5`。

### 6.3 `max_delta` 语义错误

旧实现把从 grip 参考点开始的累计位移截到 0.08 m。这不是单周期限制，而是把机械臂永久限制在参考点附近固定半径。

新实现正常模式取消累计位移截断，改为：

```python
step = raw_target - previous_command
max_step = max_speed_mps * dt
if norm(step) > max_step:
    step = normalize(step) * max_step
target = previous_command + step
```

`dt` 来自 `time.monotonic()` 的实际周期，并裁剪到 0.005–0.05 s。默认 `max_speed=0.08 m/s`。

只有 `--safe-test` 额外限制 grip 相对活动范围为 3 cm，这一范围不影响正常模式。

### 6.4 MOVEL 造成路径积累

旧代码在每帧执行：

```python
MotionCtrl_2(0x01, 0x02, 50, 0x00)
EndPoseCtrl(...)
```

MOVEL 用于直线路径，高频提交目标可能让固件继续执行已有路径段，造成延迟和停止后的追赶。

新脚本默认 MOVEP：

- Piper 启动完成后设置一次 MOVEP；
- grip rising edge 再确认一次；
- 50 Hz 循环主要只发最新 `EndPoseCtrl`；
- 不在每帧重建对象或重新初始化接口。

### 6.5 循环频率漂移

旧代码使用 `time.sleep(0.01)`，实际周期包含读取、计算、CAN 发送和额外 10 ms，并非稳定 100 Hz。

新代码采用 50 Hz deadline loop：

```python
next_tick += period
remaining = next_tick - time.monotonic()
```

超期时记录 overrun，并从当前 monotonic 时间重建 deadline，避免累计漂移。

### 6.6 未处理重复和过期 XR 数据

旧代码不读取 `get_time_stamp_ns()`，会重复处理相同 XR 帧，也无法识别断流。

新逻辑：

- timestamp 变化：使用最新 pose 计算目标；
- timestamp 相同：不产生重复末端目标；
- trigger 有变化时仍可按节流规则更新夹爪；
- timestamp 超过 0.2 s 不变：标记 stale、清参考、停止发送新末端目标；
- XR 恢复后不能在 grip 仍握住时自动恢复；
- 必须松开，观察到新帧后重新握住。

### 6.7 固定姿态

旧代码始终发送：

```python
RX = -179900
RY = 0
RZ = -179900
```

新脚本在 grip rising edge 调用 `GetArmEndPoseMsgs()`，保存实际 X/Y/Z 和 RX/RY/RZ。该次 grip 持续期间只更新 position，始终保持激活时的实际姿态。

若反馈为空、时间戳无效、反馈 Hz 无效或 position 非有限值：

- 不发送末端运动目标；
- 打印明确错误；
- 要求松开并重新 grip。

### 6.8 Grip clutch

旧代码只有 `grip > 0.5` 单阈值。新代码加入滞回：

| 状态 | 阈值 |
| --- | ---: |
| 激活 | ≥ 0.80 |
| 释放 | ≤ 0.60 |

0.60–0.80 区间保持原状态，避免阈值附近抖动。

状态行为：

| 事件 | 行为 |
| --- | --- |
| Rising | 等待最新 XR 帧，读取实际 Piper 位姿，保存参考，确认 MOVEP |
| 第一目标 | 严格等于当前实际末端位置，避免握住瞬间跳变 |
| Held + 新帧 | 映射、工作空间裁剪、限速、发送最新位置 |
| Held + 重复帧 | 不发送重复末端目标 |
| Falling | 清参考并停止新末端目标；不回 home、不失能 |
| Stale/反馈失败 | 锁止运动并要求重新 grip |

### 6.9 工作空间

沿用旧直连脚本范围：

| 轴 | 最小 | 最大 |
| --- | ---: | ---: |
| X | 0.10 m | 0.35 m |
| Y | -0.20 m | 0.20 m |
| Z | 0.10 m | 0.35 m |

处理顺序为先裁剪 `raw_target`，再按上一次发送目标做速度限制。

### 6.10 夹爪

保留旧方向：

```text
trigger=0.0 → 打开至 70000
trigger=1.0 → 关闭至 0
```

发送条件：

- trigger 相对上次发送值变化超过 0.01；或
- 距上次发送达到 0.2 s，进行静态刷新。

`--no-gripper` 可彻底禁止夹爪命令。没有自动做夹爪回零。

## 7. 安全模式与参数

### 7.1 `--safe-test`

| 参数 | 安全值 |
| --- | ---: |
| `position_scale` | 0.5 |
| `max_speed` | 0.03 m/s |
| grip 相对活动半径 | 0.03 m |

### 7.2 命令行参数

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| `--can-name` | `can0` | CAN 接口 |
| `--control-rate` | `50` | 控制频率 Hz |
| `--position-scale` | `1.0` | 位置比例 |
| `--max-speed` | `0.08` | 目标限速 m/s |
| `--speed-percent` | `40` | MOVEP 速度百分比 |
| `--yaw-deg` | `0` | 安装 yaw |
| `--safe-test` | 关闭 | 首次实机限制 |
| `--no-gripper` | 关闭 | 禁止夹爪命令 |
| `--dry-run` | 默认 | 内置无硬件模拟 |
| `--hardware` | 关闭 | 显式进入硬件确认 |

## 8. 硬件安全门

未指定 `--hardware` 时，程序强制 dry-run，并且不会：

- import `xrobotoolkit_sdk`；
- import/实例化 `C_PiperInterface_V2`；
- 调用 `xrt.init()`；
- 连接 CAN；
- 调用 Piper 或夹爪命令。

指定 `--hardware` 后先打印醒目警告，并要求输入：

```text
ARM
```

输入不完全匹配就退出，此时仍未导入硬件 SDK、未打开接口。

确认后才允许连接、使能、设置 MOVEP 和按 clutch 发送目标。启动不自动回 home、不发送固定初始位置、不关节回零。退出不发送 home、`DisableArm` 或 `ResetPiper`；只停止更新并关闭 XR 客户端，机械臂保持已使能状态，由现场既有安全流程处理。

## 9. 无硬件验证结果

### 9.1 语法

```bash
python -m py_compile pico_teleop_piper_fixed.py
```

结果：通过。

### 9.2 空白与差异

```bash
git diff --check
git diff --cached --check
```

结果：通过。

### 9.3 Dry-run

```bash
python pico_teleop_piper_fixed.py --dry-run
```

验证内容：

- 程序自动启动和退出；
- 映射矩阵不是 identity；
- 第一帧 target 等于模拟实际 EE；
- 速度限制实际触发；
- target 全部 finite；
- deadline 频率接近 50 Hz。

实际摘要：

```text
mapping_non_identity=True
first_target_no_jump=True
speed_limit_observed=True
finite=True
target_rate=50.0Hz
overruns=0
result=PASS
```

## 10. 第一次真实测试

本次未执行以下命令。现场执行前必须确认工作空间无人无障碍、急停可用、CAN/Pico 正常，并由人员准备立即停止系统。

### 10.1 首次 commissioning

```bash
cd /root/data/piper_teleop
python pico_teleop_piper_fixed.py \
    --hardware \
    --safe-test \
    --can-name can0 \
    --yaw-deg 0
```

确认现场安全后才输入 `ARM`。初次建议加 `--no-gripper`，先单独验证机械臂方向。

### 10.2 Yaw 排查

保持 `--safe-test`，每次只做小幅单轴动作。默认应满足：

- XR X → Piper -Y；
- XR Y → Piper +Z；
- XR Z → Piper -X。

若安装方向不同，停止程序后分别尝试：

```text
--yaw-deg 90
--yaw-deg -90
--yaw-deg 180
```

每次修改 yaw 后都重新启动和建立 grip 参考，不在 grip 持续期间切换。

### 10.3 正常模式

只有在方向、停止行为、无跳变和工作空间全部确认后使用：

```bash
cd /root/data/piper_teleop
python pico_teleop_piper_fixed.py \
    --hardware \
    --can-name can0 \
    --position-scale 1.0 \
    --max-speed 0.08 \
    --speed-percent 40 \
    --yaw-deg 0
```

## 11. 硬件验收清单

必须重点观察：

1. 三个方向是否正确，有无串轴或比例异常；
2. 手柄停下后机械臂是否仍长时间继续追赶；
3. 松开再握住 grip 时是否发生位置或姿态跳变。

建议同时记录：

- 第一条目标是否保持实际位置；
- 快速移动后最大追赶距离和停止时间；
- XR 断流 0.2 s 后是否停止新目标；
- XR 恢复但 grip 未松开时是否继续锁止；
- 重新 grip 后是否从新的实际 EE 建立参考；
- Piper 末端反馈 Hz 是否稳定；
- 工作空间边界是否出现突兀贴边运动；
- trigger 方向是否仍为 0=开、1=闭。

## 12. 回退

新脚本为独立文件，停止 `pico_teleop_piper_fixed.py` 即可停止使用新路径。旧 `pico_teleop_piper.py` 和 `pico_teleop_improved.py` 完全保留。

注意：旧 `pico_teleop_piper.py` 启动会自动移动到固定 home，并保留 MOVEL、固定姿态和无 timestamp 等旧行为。因此旧文件可用于代码回溯，但不能在未知现场直接当作安全回退命令运行。

控制实现 commit 为 `043b32740fb70110f9367a722cae9596494dcaba`。如需撤销，应由维护人员在确认依赖后使用常规 `git revert`。本次未执行任何 destructive Git 操作。

## 13. 剩余风险

以下只能通过真实硬件确认：

- Pico 实际安装 yaw；
- Piper 固件是否在 MOVEP 下稳定覆盖旧目标；
- 0.08 m/s 和 40% 是否适合当前负载；
- 三组末端反馈 CAN 帧与反馈 Hz 是否稳定；
- 既有工作空间是否适合当前安装；
- 0.2 s stale 阈值是否适合现场 XR 更新率；
- 速度限制引入的短距离正常滞后是否可接受；
- 夹爪实际行程是否仍为 70 mm。

这些参数应基于 `--safe-test` 现场记录做小幅调整，不建议在本轮基础上立即扩展为 ROS2、Placo 或复杂预测控制重构。

## 14. 结论

本次修复直接处理了坐标轴错误、比例过激、`max_delta` 语义错误、MOVEL 路径积累、控制周期漂移、重复/过期 XR 帧、固定末端姿态和 grip 抖动。

软件侧最小修复与无硬件检查已经完成。最终现场效果应以三项核心验收现象为准：三轴方向、停止后追赶、重新 grip 无跳变。

## 15. 代码级变更索引

### 15.1 旧脚本问题位置

| 文件与行号 | 原行为 | 影响 |
| --- | --- | --- |
| `pico_teleop_piper.py:108` | `position_scale=3.5` | 放大控制器噪声和错误轴向 |
| `pico_teleop_piper.py:109` | `max_delta=0.08` | 名称暗示单步限制，实际用于累计相对半径 |
| `pico_teleop_piper.py:115-121` | 启动自动移动到固定 home | 未知现场姿态下存在额外启动运动 |
| `pico_teleop_piper.py:135-140` | Grip 时只读取实际 X/Y/Z | 没有保存实际 RX/RY/RZ |
| `pico_teleop_piper.py:148` | XR 三轴直接相减 | 没有官方 XR→机器人坐标变换 |
| `pico_teleop_piper.py:150-153` | 裁剪累计 delta 模长 | 永久限制在 grip 参考点 8 cm 内 |
| `pico_teleop_piper.py:172-174` | 每帧重发 MOVEL 和固定姿态 | 容易积累路径，且姿态可能跳变 |
| `pico_teleop_piper.py:207-219` | 每轮读取但不检查 timestamp | 重复帧与 stale 帧无法区分 |
| `pico_teleop_piper.py:222` | `grip > 0.5` 单阈值 | 阈值附近容易抖动 |
| `pico_teleop_piper.py:250-251` | 固定 `sleep(0.01)` | 执行耗时叠加，周期漂移 |

### 15.2 新脚本实现位置

| 文件与行号 | 实现 |
| --- | --- |
| `pico_teleop_piper_fixed.py:18-25` | 官方 XR→Piper 基础矩阵 |
| `pico_teleop_piper_fixed.py:27-34` | 工作空间、clutch、stale、单位常量 |
| `pico_teleop_piper_fixed.py:37-49` | 安装 yaw 旋转矩阵 |
| `pico_teleop_piper_fixed.py:52-64` | 基于实际 `dt` 的单周期速度限制 |
| `pico_teleop_piper_fixed.py:67-72` | 仅 safe-test 使用的 3 cm 半径限制 |
| `pico_teleop_piper_fixed.py:75-82` | Grip 0.80/0.60 滞回和边沿检测 |
| `pico_teleop_piper_fixed.py:85-129` | CLI、安全默认值和参数校验 |
| `pico_teleop_piper_fixed.py:132-191` | 延迟导入的 Piper SDK 薄封装 |
| `pico_teleop_piper_fixed.py:194-296` | 内置 dry-run 轨迹和断言 |
| `pico_teleop_piper_fixed.py:299-310` | `--hardware` 的 `ARM` 二次确认 |
| `pico_teleop_piper_fixed.py:329-350` | 50 Hz deadline 与真实 `dt` |
| `pico_teleop_piper_fixed.py:352-389` | XR timestamp、新帧与输入有效性 |
| `pico_teleop_piper_fixed.py:391-410` | Rising edge、实际反馈和第一帧无跳变 |
| `pico_teleop_piper_fixed.py:411-430` | Held 状态的映射、裁剪、限速和发送 |
| `pico_teleop_piper_fixed.py:436-447` | 0.2 s stale 锁止 |
| `pico_teleop_piper_fixed.py:449-460` | 夹爪变化阈值和 0.2 s 刷新 |
| `pico_teleop_piper_fixed.py:462-469` | deadline sleep 与 overrun 处理 |
| `pico_teleop_piper_fixed.py:470-475` | 不 home、不 reset、不 disable 的退出行为 |

行号基于控制实现 commit `043b32740fb70110f9367a722cae9596494dcaba`。

## 16. 端到端执行时序

### 16.1 默认启动

```text
main
└─ parse_args
   ├─ 未提供 --hardware
   ├─ 强制 args.dry_run=True
   ├─ 计算 R_FINAL
   └─ run_dry_run
      ├─ 不导入 XR SDK
      ├─ 不导入 Piper SDK
      ├─ 运行 2.4 s 模拟轨迹
      ├─ 打印验证摘要
      └─ 自动退出
```

即使只执行 `python pico_teleop_piper_fixed.py`，也不会触发硬件。

### 16.2 硬件模式启动

```text
main
└─ parse_args(--hardware)
   ├─ 打印硬件警告
   ├─ 等待输入 ARM
   ├─ 输入不匹配 → return 2，无 SDK import
   └─ 输入匹配
      ├─ import xrobotoolkit_sdk
      ├─ 构造 PiperHardware
      │  ├─ import C_PiperInterface_V2
      │  ├─ ConnectPort
      │  ├─ EnablePiper，最多尝试 100 次
      │  └─ 设置一次 MOVEP
      ├─ xrt.init
      └─ 进入 50 Hz loop
```

所有连接、使能和命令调用都位于 `ARM` 确认之后。

### 16.3 每个 50 Hz 周期

循环每次依次执行：

1. 读取 `cycle_start=time.monotonic()`；
2. 推进下一 deadline；
3. 用本周期实际间隔计算 `dt`；
4. 读取 XR timestamp、grip、trigger；
5. 更新 grip hysteresis 与 rising/falling edge；
6. falling 时立即清空所有运动参考；
7. 仅 timestamp 变化时读取 controller pose；
8. 校验 pose 形状必须为 3 且全部 finite；
9. rising pending 时读取 Piper 实际末端反馈；
10. held 时计算最新相对目标；
11. 独立检查 0.2 s stale；
12. 按节流条件处理夹爪；
13. 睡眠到 deadline，或记录 overrun。

### 16.4 Grip rising edge

Rising edge 可能发生在两个 XR pose frame 之间，因此代码不会用重复 pose 立即标定，而是设置 `activation_pending=True`，等待下一条新 timestamp。

收到新帧后：

```text
读取完整 Piper end pose
→ 保存 ee_reference
→ 保存 held_orientation
→ 保存转换后的 xr_reference_robot
→ previous_command=ee_reference
→ 再确认 MOVEP
→ EndPoseCtrl(实际 EE position, 实际 RX/RY/RZ)
```

因此第一条末端目标在数值上等于实际反馈，不包含手柄绝对坐标，也不包含旧固定姿态。

### 16.5 Grip held

只有以下条件全部成立才发送末端目标：

```text
grip_active
AND not require_regrip
AND xr_reference_robot is not None
AND ee_reference is not None
AND held_orientation is not None
AND previous_command is not None
AND 当前 XR timestamp 是新帧
```

任意条件不满足时，本周期不会发送 `EndPoseCtrl`。

### 16.6 Grip falling edge

Falling edge 会清除：

- `activation_pending`；
- `xr_reference_robot`；
- `ee_reference`；
- `held_orientation`；
- `previous_command`；
- `require_regrip`。

它不会发送 home、停止模式、Disable 或 Reset。效果是停止生成新目标，并让下一次 rising 从当时实际 EE 重新建立参考。

## 17. 控制状态变量说明

| 变量 | 类型 | 含义 | 何时清除 |
| --- | --- | --- | --- |
| `grip_active` | bool | 滞回后的 clutch 状态 | grip ≤ 0.60 |
| `activation_pending` | bool | 已检测 rising，等待下一条新 XR pose | 成功激活、falling 或 XR pose 非法 |
| `require_regrip` | bool | 安全锁止，禁止运动直到释放并恢复新帧 | grip released 且收到 fresh XR |
| `stale_announced` | bool | 防止每周期重复打印 stale | 收到新 timestamp |
| `last_xr_timestamp` | int/SDK 类型 | 上一条已处理 XR timestamp | 进程退出 |
| `last_new_xr_time` | float | 最近新 timestamp 的 monotonic 时间 | 每条新 timestamp |
| `xr_reference_robot` | 3-vector | Grip 激活时已转换 XR position | falling、stale、错误 |
| `ee_reference` | 3-vector | Grip 激活时实际 Piper position | falling、stale |
| `held_orientation` | 3-int tuple | Grip 激活时实际 RX/RY/RZ | falling、stale |
| `previous_command` | 3-vector | 上次实际发送的 position target | falling、stale |
| `previous_trigger` | float | 上次实际发送的 trigger 值 | 进程退出 |
| `last_gripper_send` | float | 上次夹爪发送时间 | 每次夹爪发送 |
| `overrun_count` | int | deadline 超时次数 | 进程退出 |

`require_regrip` 是防止断流自动恢复后突然移动的关键锁存。只收到新 timestamp 并不足以解除正在握持时的锁止。

## 18. Yaw 矩阵与轴向表

### 18.1 四种安装角度

基础映射后再绕 Piper Z 轴旋转，得到：

| `--yaw-deg` | Piper X | Piper Y | Piper Z |
| ---: | --- | --- | --- |
| 0 | `-XR Z` | `-XR X` | `XR Y` |
| 90 | `XR X` | `-XR Z` | `XR Y` |
| -90 | `-XR X` | `XR Z` | `XR Y` |
| 180 | `XR Z` | `XR X` | `XR Y` |

该 yaw 只修正整套设备的平面安装朝向，不使用手柄 orientation，也不会改变 position-only 设计。

### 18.2 数值示例

假设 grip 后手柄相对移动：

```text
delta_XR = [+0.020, -0.010, -0.030] m
```

Yaw 0、scale 1.0 时：

```text
delta_Piper = [+0.030, -0.020, -0.010] m
```

Safe-test 的 scale 0.5 时：

```text
delta_Piper = [+0.015, -0.010, -0.005] m
```

如果当前实际 EE 为 `[0.200, 0.000, 0.200] m`，未触发工作空间或速度限制时，raw target 分别是：

```text
normal:    [0.230, -0.020, 0.190] m
safe-test: [0.215, -0.010, 0.195] m
```

## 19. 时间与速度限制计算细节

### 19.1 默认 50 Hz

```text
period = 1 / 50 = 0.020 s
```

正常模式：

```text
max_step = 0.08 m/s × 0.020 s = 0.0016 m = 1.6 mm/周期
```

Safe-test：

```text
max_step = 0.03 m/s × 0.020 s = 0.0006 m = 0.6 mm/周期
```

### 19.2 `dt` 裁剪的影响

| `dt` | 正常最大步长 | Safe-test 最大步长 |
| ---: | ---: | ---: |
| 0.005 s | 0.4 mm | 0.15 mm |
| 0.020 s | 1.6 mm | 0.60 mm |
| 0.050 s | 4.0 mm | 1.50 mm |

裁剪下限避免异常短周期把允许步长压到近零；上限避免单次系统停顿后产生过大的补偿步长。

### 19.3 限速示例

若：

```text
previous_command = [0.2000, 0.0000, 0.2000]
raw_target       = [0.2100, 0.0000, 0.2000]
dt               = 0.020 s
max_speed        = 0.08 m/s
```

请求步长为 10 mm，但本周期只允许 1.6 mm，因此发送：

```text
target = [0.2016, 0.0000, 0.2000]
```

### 19.4 对“追赶”的边界说明

MOVEP 和最新 timestamp 处理用于减少旧 MOVEL 路径积累，但软件速度限制本身仍会产生有界的正常跟随滞后：如果手柄移动速度长期高于 `max_speed`，`previous_command` 会落后于 `raw_target`，手柄停止后目标仍可能用若干周期追到最终 raw target。

因此现场应区分：

- 异常现象：长时间执行旧 MOVEL 路径、停止距离不可控；
- 预期现象：由 0.08 m/s 限速造成的短距离、可计算追随。

若剩余误差为 16 mm，理论最短追随时间约为：

```text
0.016 m / 0.08 m/s = 0.20 s
```

若这种短追随也不可接受，应先基于现场数据调整 `max-speed` 与操作手势，而不是重新启用 MOVEL。

## 20. 异常与恢复矩阵

| 异常 | 检测 | 末端命令 | 恢复条件 |
| --- | --- | --- | --- |
| XR timestamp 重复 | 与上一 timestamp 相同 | 本周期不发 | 下一条新 timestamp |
| XR stale | 0.2 s 无新 timestamp | 清参考并禁止 | 先松开，收到新帧，再 grip |
| XR position 形状错误 | shape 不是 `(3,)` | 禁止 | 松开并重新 grip |
| XR position NaN/Inf | `np.isfinite` 失败 | 禁止 | 松开并重新 grip |
| Piper feedback 为空 | getter 返回 None | 禁止 | 松开并重新 grip |
| Piper feedback timestamp≤0 | 尚无有效反馈 | 禁止 | 反馈稳定后重新 grip |
| Piper feedback Hz≤0 | 三组末端帧不完整 | 禁止 | 反馈稳定后重新 grip |
| Target NaN/Inf | 发送前有限性检查失败 | 禁止并清 XR 参考 | 松开并重新 grip |
| EnablePiper 失败 | 100 次内未成功 | 抛错，不发位置目标 | 检查现场连接后重启 |
| Deadline overrun | remaining≤0 | 当前计算已完成；重建 deadline | 系统负载恢复 |
| Ctrl+C | KeyboardInterrupt | 停止新目标 | 人员按现场流程处理已使能机械臂 |

夹爪是独立节流路径。重复 XR timestamp 时仍可处理 trigger；`--no-gripper` 可完全关闭该路径。

## 21. 硬件 API 调用审计

### 21.1 Dry-run 可达调用

Dry-run 只使用 argparse、math、time、typing 和 NumPy。以下符号在 dry-run 路径不可达：

```text
xrt.init
C_PiperInterface_V2
ConnectPort
EnablePiper
MotionCtrl_2
EndPoseCtrl
GripperCtrl
```

Piper 和 XR import 均位于硬件函数内部，而不是模块顶层。

### 21.2 硬件模式调用顺序

输入 `ARM` 后，允许调用顺序为：

1. import XR SDK；
2. import Piper SDK；
3. 构造接口；
4. `ConnectPort`；
5. `EnablePiper`；
6. 启动时一次 `MotionCtrl_2(... MOVEP ...)`；
7. `xrt.init`；
8. grip rising 时读取反馈并再次确认 MOVEP；
9. grip active 且新 timestamp 时 `EndPoseCtrl`；
10. grip active 且满足节流时 `GripperCtrl`；
11. 退出时 `xrt.close`。

明确不存在：

- `JointCtrl`；
- 自动 home；
- 自动关节回零；
- `DisableArm`；
- `ResetPiper`；
- CAN 激活脚本；
- ROS launch。

## 22. Dry-run 轨迹细节

内置模拟总时长 2.4 s：

| 时间 | Grip | XR 动作 |
| ---: | --- | --- |
| 0.00–0.20 s | 松开 | 保持基准 pose |
| 约 0.20 s | Rising | 保存参考，第一 target=实际 EE |
| 0.20–0.80 s | Held | XR X 以 0.12 m/s 增加 |
| 0.80–1.40 s | Held | XR X 保持，XR Z 以 0.10 m/s 减少 |
| 1.40–2.20 s | Held | 手柄停止，目标收敛并保持 |
| 约 2.20 s | Falling | 清参考，停止生成目标 |
| 2.20–2.40 s | 松开 | 无目标 |
| 2.40 s | — | 自动退出 |

模拟速度刻意高于默认 0.08 m/s，以确保 `speed_limit_observed=True`。模拟基准 EE 为 `[0.20, 0.00, 0.20] m`，位于工作空间内。

`target_rate` 的统计分母扣除了 grip 前后合计约 0.4 s 的非目标时段，因此验证的是 active target 更新率，而不是整个程序平均输出率。

## 23. 参数调优顺序

现场不要同时修改多个参数。建议按以下顺序：

1. 保持 `--safe-test --no-gripper`，只确认 yaw 和三个轴方向；
2. 保持 yaw，确认重新 grip 第一帧无跳变；
3. 快速移动后停住，测量追赶距离和时间；
4. 调整 `--max-speed`，确认软件目标步长；
5. 再评估 `--speed-percent` 是否限制固件实际跟随；
6. 最后调整 `--position-scale`；
7. 机械臂稳定后再启用夹爪。

三个主要参数的职责不同：

| 参数 | 控制层 | 主要影响 |
| --- | --- | --- |
| `position-scale` | XR 映射 | 手柄位移对应的最终目标距离 |
| `max-speed` | Python 目标生成 | 相邻发送目标的最大变化率 |
| `speed-percent` | Piper 固件 | 机械臂执行 MOVEP 的速度比例 |

如果方向错误，不能通过 scale 或 speed 修正，应先选择正确 yaw。

## 24. 运行日志解释

| 日志前缀 | 含义 | 操作 |
| --- | --- | --- |
| `DRY-RUN` | 无硬件模拟路径 | 可安全检查 |
| `[clutch] activated` | 已读取实际 EE 并建立参考 | 可开始小幅移动 |
| `[clutch] released` | 已停止生成目标并清参考 | 下次可重新 grip |
| `[XR ERROR]` | 控制器 position 非法 | 松开，检查 XR，重新 grip |
| `[PIPER ERROR]` | 末端反馈不完整 | 等反馈稳定后重新 grip |
| `[TARGET ERROR]` | 目标出现非有限值 | 停止测试并检查输入 |
| `[XR STALE]` | 超过 0.2 s 无新 XR 帧 | 松开，等待恢复，再 grip |
| `[timing]` | deadline overrun 计数 | 检查服务器负载 |

`[clutch] activated` 打印的位置是以米为单位的实际 EE reference，不是手柄绝对位置。

## 25. 现场数据记录模板

建议每个 yaw 单独记录一行：

| 日期/操作者 | 固件 | yaw | safe-test | scale | max-speed | speed% | X方向 | Y方向 | Z方向 | 停止追赶距离 | 停止时间 | 重新grip跳变 | XR stale恢复 | 备注 |
| --- | --- | ---: | --- | ---: | ---: | ---: | --- | --- | --- | ---: | ---: | --- | --- | --- |
|  |  | 0 | 是 | 0.5 | 0.03 | 40 |  |  |  |  |  |  |  |  |

最低验收建议：

- 三轴方向全部符合选定 yaw；
- 重新 grip 不出现肉眼可见跳变；
- XR stale 后不自动恢复运动；
- 停止追赶距离和时间可重复、可接受；
- 无 `PIPER ERROR`、`TARGET ERROR`；
- deadline overrun 不持续增长。

## 26. 有意未实现的内容

为保持快速修复边界，本次有意没有实现：

- 手柄 orientation 跟随；
- 自动坐标标定；
- 加速度/jerk 限制；
- 运动预测或网络延迟补偿；
- 多线程 XR 接收；
- 目标队列；
- ROS2 topic 桥接；
- Placo/PyKDL IK；
- 碰撞检测；
- 奇异点处理；
- 自动 home/disable/reset；
- GUI、RViz、Meshcat 或 OpenCV 窗口；
- 大型测试框架。

这些不是遗漏，而是为了减少首次真实机器人验证前的变化面。

## 27. Commit 与差异关系

| Commit | 内容 |
| --- | --- |
| `043b32740fb70110f9367a722cae9596494dcaba` | 新增修复脚本和初版报告 |
| `85f57923ac4d1379a1e69a296971d9274e136c65` | 将报告扩展为完整技术报告 |

控制代码只存在于第一个 commit；第二个 commit 只改 Markdown。本附录继续只修改报告，不改变已通过 dry-run 的控制实现。
