# PICO → Piper 单臂 / 双臂遥操作

本仓库提供 PICO XR 手柄到 AgileX Piper 机械臂的实时遥操作。当前代码保留三条明确分离的运行路径：可回退的 V1 笛卡尔控制、单臂 V2 关节 IK，以及单进程双臂 V2。

> **真机安全提示**：启动前清空机械臂工作区并确保急停可触及。首次连接、CAN 映射变化或代码修改后，必须先单臂、低速、小幅验证。不要在同一 CAN 接口上同时运行两个控制程序。

## 1. 当前状态

| 版本 | 状态 | 入口 | 控制方式 |
|---|---|---|---|
| V1 Cartesian Stable | `SUPPORTED`，冻结回退版 | `run_piper_normal_teleop.sh` | `EndPoseCtrl`，位置跟随、姿态保持 |
| V1 Safe Test | `SUPPORTED`，恢复检查 | `run_piper_safe_teleop.sh` | 低速、小范围、不控制夹爪 |
| V2 Single-arm IK | `LIVE TESTED` | `run_piper_v2_single.sh` | 四元数姿态 + Pinocchio J1–J6 IK |
| V2 Bimanual IK | `LIVE TESTED / EXPERIMENTAL` | `run_piper_v2_bimanual.sh` | 一个 XR 客户端、两套独立 IK、两路 CAN |
| 历史 Python/ROS/Placo 实验 | `LEGACY` | 无正式入口 | 仅用于追溯 |

2026-08-27 的工作站现场验证配置：

```text
left PICO controller  -> can0 -> left Piper
right PICO controller -> can1 -> right Piper
control rate          = 50 Hz
position scale        = 0.8
rotation scale        = 1.0
orientation range     = ±180°
joint step limit      = 5°/frame
IK position tolerance = 2 mm
Piper speed           = 100%
Python XYZ speed cap  = disabled
Python workspace clip = disabled
gripper               = binary full-open/full-close
```

双臂版本已经记录到左右 `XR READY`、左右 Grip 激活、两路关节命令以及同时遥操作日志。它目前**没有双臂自碰撞或机械臂之间的碰撞规划**，因此仍标记为实验性。

版本边界与冻结提交见 [VERSIONS.md](VERSIONS.md)，更长的历史操作说明见 [docs/OPERATIONS_ZH.md](docs/OPERATIONS_ZH.md)。

## 2. 系统架构

### 单臂 V2

```text
PICO controller pose + Grip/Trigger
             │
             ▼
relative translation + quaternion rotation
             │
             ▼
Pinocchio position-priority / soft-orientation IK
             │
             ▼
IK acceptance + per-frame joint-step limiter
             │
             ▼
Piper JointCtrl over can0
```

### 双臂 V2

```text
                    one XRoboToolkit SDK client
                              │
                   ┌──────────┴──────────┐
                   ▼                     ▼
          left controller         right controller
          independent Grip        independent Grip
                   │                     │
          left IK state           right IK state
                   │                     │
             can0 / left             can1 / right
```

双臂必须使用一个进程读取两个手柄。现场验证发现两个独立 Python XR 客户端会互相干扰，第二客户端可能收到零位姿，同时使主客户端出现 `XR STALE`。

## 3. 关键文件

```text
pico_teleop_piper_fixed.py          # V1 冻结核心
pico_teleop_piper_ik_v2.py          # V2 单臂核心
pico_teleop_piper_bimanual_v2.py    # V2 双臂核心（单 XR 客户端）
piper_ik_v2.py                       # Piper URDF / Pinocchio IK
assets/piper_description.urdf        # J1–J6 运动学模型
test_piper_ik_v2.py                  # IK、四元数和关节限幅测试
run_piper_normal_teleop.sh           # V1 正式入口
run_piper_safe_teleop.sh             # V1 安全测试入口
run_piper_v2_single.sh               # V2 单臂固定参数入口
run_piper_v2_bimanual.sh             # V2 双臂固定参数入口
```

旧文件暂不移动，避免破坏历史导入路径；它们不应作为新的真机入口。

## 4. 环境安装

### 4.1 克隆代码

```bash
git clone https://github.com/insomniapku/piper_teleop.git
cd piper_teleop
git switch feature/piper-ik-orientation-v2
```

### 4.2 Python 环境

当前现场验证版本：Python 3.12.3、NumPy 1.26.4、SciPy 1.11.4、Pinocchio 3.8.0、`piper-sdk` 0.6.2、`xrobotoolkit-sdk` 1.0.2。

```bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements-v2.txt
python -m pip install -e ./piper_sdk
python -m pip install -e .
```

`xrobotoolkit_sdk` 来自 XRoboToolkit PC Service 的 Python binding。若工作站没有该模块，应按照 XRoboToolkit 对应版本安装或编译；不要从不明来源安装同名包。

验证运行时：

```bash
python -c "import numpy, scipy, pinocchio, piper_sdk, xrobotoolkit_sdk; print('runtime imports: OK')"
```

如果虚拟环境不在仓库的 `.venv`，启动脚本支持显式指定解释器：

```bash
export PIPER_PYTHON=/absolute/path/to/venv/bin/python
```

当前工作站复用既有环境时可设置：

```bash
export PIPER_PYTHON=/home/zktitan/piper_teleop_latency/.venv/bin/python
```

## 5. CAN 配置

安装工具：

```bash
sudo apt update
sudo apt install can-utils ethtool
```

查找适配器：

```bash
cd piper_sdk/piper_sdk
bash find_all_can_port.sh
```

单臂示例：

```bash
bash can_activate.sh can0 1000000
```

双臂必须把两个 USB-CAN 适配器稳定映射为不同接口。当前工作站验证映射为：

| 机械臂 | CAN | USB 路径 | 适配器序列号 |
|---|---|---|---|
| 左臂 | `can0` | `1-11.2:1.0` | `002300365547570420303135` |
| 右臂 | `can1` | `1-11.1:1.0` | `002F003D5547571120343930` |

```bash
bash can_activate.sh can0 1000000 "1-11.2:1.0"
bash can_activate.sh can1 1000000 "1-11.1:1.0"
```

每次重插 USB 或重启工作站后都应验证，不要只相信接口名：

```bash
ip -details link show can0
ip -details link show can1
timeout 2 candump can0
timeout 2 candump can1
```

正常情况下两台 Piper 都会持续广播反馈；当前现场观测约为 200 Hz。

## 6. PICO / XRoboToolkit 连接

1. 在 Linux 工作站上启动一个且仅一个 XRoboToolkit PC Service。
2. 在 PICO 中打开 XRoboToolkit 应用并保持前台运行。
3. PICO 连接到 Linux 工作站 IP 和 PC Service **当前实际监听端口**。
4. 确认左右手柄姿态时间戳持续更新，再按 Grip。

查找监听端口：

```bash
ss -lntp | grep RoboticsService
```

当前工作站地址为 `192.168.111.123`，2026-08-27 的服务监听端口为 `63901`。历史运行曾出现其他端口，所以不要把端口永久写死；以当次 `ss` 输出或 PC Service 界面为准。

不要启动第二个 PC Service，也不要并行运行 `diagnose_xr_connection.py` 与正式遥操进程。二者都可能抢占 XR 数据流。

## 7. 启动前检查

```bash
ip -brief link | grep can
pgrep -af 'pico_teleop|RoboticsServiceProcess'
ss -lntp | grep RoboticsService
```

确认：

- 机械臂周围无人、无线缆和障碍物；
- 急停可立即触及；
- `can0/can1` 映射与左右机械臂一致；
- 没有其他遥操、SDK demo 或 JointCtrl 程序；
- PICO 应用在前台且两个 Grip 都处于松开状态。

## 8. 启动 V2 单臂

```bash
cd /path/to/piper_teleop
PIPER_PYTHON=/path/to/venv/bin/python ./run_piper_v2_single.sh
```

程序要求输入 `ARM` 后才连接 CAN 并使能机械臂。启动后：

- 左 Grip 按住：开始跟随；
- 左 Grip 松开：立即停止更新关节目标并允许重新摆放手柄；
- 左 Trigger：完整张开或完整闭合夹爪。

等价完整命令：

```bash
python -u pico_teleop_piper_ik_v2.py \
  --hardware \
  --controller-hand left \
  --position-scale 0.8 \
  --rotation-scale 1.0 \
  --max-orientation-delta-deg 180 \
  --max-joint-step-deg 5 \
  --ik-position-tolerance-mm 2 \
  --no-speed-limit \
  --no-workspace-limit \
  --speed-percent 100 \
  --binary-gripper \
  --can-name can0 \
  --yaw-deg 0
```

## 9. 启动 V2 双臂

```bash
cd /path/to/piper_teleop
PIPER_PYTHON=/path/to/venv/bin/python ./run_piper_v2_bimanual.sh
```

程序要求输入 `BIMANUAL` 后才连接两路 CAN。

控制映射：

| 输入 | 输出 |
|---|---|
| 左 Grip | 启停 `can0` 左臂跟随 |
| 左 Trigger | 左夹爪完整开/闭 |
| 右 Grip | 启停 `can1` 右臂跟随 |
| 右 Trigger | 右夹爪完整开/闭 |

第一次验证顺序：

1. 两个 Grip 都松开；
2. 只按左 Grip，分别做小幅前后、左右、上下和旋转；
3. 松开左 Grip；
4. 只按右 Grip，重复方向验证；
5. 两边方向正确后才同时按住左右 Grip；
6. 双臂末端保持足够间距，因为当前没有双臂碰撞检测。

等价完整命令：

```bash
python -u pico_teleop_piper_bimanual_v2.py \
  --hardware \
  --left-can-name can0 \
  --right-can-name can1 \
  --position-scale 0.8 \
  --rotation-scale 1.0 \
  --max-orientation-delta-deg 180 \
  --max-joint-step-deg 5 \
  --ik-position-tolerance-mm 2 \
  --no-speed-limit \
  --no-workspace-limit \
  --speed-percent 100 \
  --binary-gripper \
  --left-yaw-deg 0 \
  --right-yaw-deg 0
```

## 10. Grip、夹爪和安全机制

### Grip clutch

- Grip ≥ `0.80`：激活该手对应机械臂；
- Grip ≤ `0.60`：释放并停止关节目标更新；
- 每次激活都读取真机当前关节角，首个目标等于测量状态，避免重抓取时跳变。

Grip 是安全离合器，不是夹爪按键。Trigger 才控制夹爪。

### 夹爪

带 `--binary-gripper` 时：

- Trigger ≤ `0.40`：完整张开；
- Trigger ≥ `0.60`：完整闭合；
- 中间区间保持上一状态，避免抖动；
- 完整张开命令为 `100000` SDK units。

### XR 超时

- 超过 `0.2 s` 没有新 XR 时间戳：停止更新；
- 恢复后要求累计 10 个有效帧；
- 双臂模式中 XR 全局断流会同时锁住两臂。

### IK HOLD

V2 的位置接受容差为 `2 mm`。小于该误差的有限解可以继续发送；不可达、超过容差、求解停滞或触碰 URDF 关节边界的目标保持上一条命令。不要完全删除 IK HOLD，它用于阻止错误解进入真机。

### 关节步长限制

当前现场值为 `5°/frame`。在 50 Hz 控制循环下，它是应用层最后一道关节目标变化限制。日志曾观察到原始 IK 请求 20–100° 的跳变，因此不要在真机上使用 `--no-joint-step-limit`。

`--no-speed-limit` 和 `--no-workspace-limit` 只关闭 Python 层的笛卡尔速度裁剪和 XYZ 工作空间裁剪；URDF 关节边界、IK HOLD、5°关节步长、Piper 固件保护与 Grip/XR 超时仍然存在。

## 11. tmux 后台运行

单臂：

```bash
mkdir -p logs
tmux new -s piper_v2_single
./run_piper_v2_single.sh 2>&1 | tee logs/v2_single.log
```

双臂：

```bash
mkdir -p logs
tmux new -s piper_v2_bimanual
./run_piper_v2_bimanual.sh 2>&1 | tee logs/v2_bimanual.log
```

分离会话：`Ctrl-B`，再按 `D`。恢复：

```bash
tmux attach -t piper_v2_bimanual
```

停止时先松开两个 Grip，再在对应 tmux 窗口按 `Ctrl-C`。程序退出不会自动归零、回 Home 或 Disable；这是为了避免退出时产生额外运动。

不要使用 `pkill python`。检查精确进程：

```bash
pgrep -af 'pico_teleop_piper_(ik_v2|bimanual_v2).py'
```

## 12. 离线测试

测试不会连接 XR 或 CAN：

```bash
python -m py_compile \
  pico_teleop_piper_ik_v2.py \
  pico_teleop_piper_bimanual_v2.py \
  piper_ik_v2.py

python -m unittest test_piper_ik_v2.py

python pico_teleop_piper_ik_v2.py \
  --dry-run \
  --position-scale 0.8 \
  --rotation-scale 1.0 \
  --max-orientation-delta-deg 180 \
  --max-joint-step-deg 5 \
  --ik-position-tolerance-mm 2 \
  --no-speed-limit \
  --no-workspace-limit

python pico_teleop_piper_bimanual_v2.py --dry-run \
  --position-scale 0.8 \
  --rotation-scale 1.0 \
  --max-orientation-delta-deg 180 \
  --max-joint-step-deg 5 \
  --ik-position-tolerance-mm 2 \
  --no-speed-limit \
  --no-workspace-limit
```

双臂 dry-run 会分别验证左右 121 帧轨迹。

## 13. 日志含义

| 日志 | 含义 | 操作 |
|---|---|---|
| `XR READY` | 连续 10 帧有效，可按 Grip | 正常 |
| `XR ERROR` | 位姿全零、非法或四元数无效 | 检查 PICO 应用/连接 |
| `XR STALE` | 0.2 秒没有新数据 | 松开 Grip，恢复 XR |
| `clutch activated` | 已读取真机关节并建立新参考 | 可小幅移动 |
| `clutch released` | 已停止目标更新 | 正常 |
| `IK HOLD` | 当前目标不可接受，保持上一命令 | 减小动作/重新 clutch |
| `JOINT LIMIT` | 原始 IK 步长超过 5° | 放慢手柄，观察是否持续饱和 |
| `PIPER ERROR` | 没有完整关节反馈 | 检查 CAN 和机械臂供电 |

## 14. 常见问题

### PICO 显示连接但机械臂不动

1. 检查 PICO 应用是否在前台；
2. 用 `ss -lntp | grep RoboticsService` 确认实际端口；
3. 检查日志是否出现 `XR READY`；
4. 确认对应 Grip 超过激活阈值；
5. 确认程序没有停留在 `ARM`/`BIMANUAL` 确认提示。

### 单臂正常，双臂没有数据

不要启动两份单臂脚本。停止它们和诊断客户端，只保留一个 PC Service，然后启动 `pico_teleop_piper_bimanual_v2.py`。

### 运动一卡一卡

查看 `IK HOLD`、`JOINT LIMIT` 和 `XR STALE`：

- `IK HOLD` 多：目标不可达、姿态要求过强或接近关节边界；
- `JOINT LIMIT` 多：手柄移动过快或 IK 分支跳变，机械臂正在追赶；
- `XR STALE`：PICO 网络或应用前台状态不稳定；
- 频繁 `clutch released` 且操作者未松手：Grip 模拟量可能抖动。

### 左右/前后方向反了

先确认手柄、CAN 和机械臂映射。当前现场参数左右均为 yaw `0`。不同安装朝向可分别调整 `--left-yaw-deg` 或 `--right-yaw-deg`，每次只改一个变量并做小幅单轴验证。

### 夹爪只半开

确认命令包含 `--binary-gripper`，并确认运行的是 V2 正式启动脚本。当前完整张开目标是 `100000` units。

## 15. 已知限制

- 双臂之间没有碰撞检测或协同规划；
- IK 仍可能在奇异位形或关节边界附近 HOLD；
- 原始 IK 偶尔可能产生大关节跳变，当前依靠 5°/frame 阻挡；
- XR SDK 只提供全局时间戳，无法独立判断某一只静止手柄是否单独断流；
- 当前 V2 使用软姿态目标，不保证所有位置都能实现完整末端朝向；
- 日志尚未包含端到端位姿年龄和每周期 IK 耗时统计。

## 16. 发布与贡献规则

- 不提交密码、令牌、私钥、日志、录制数据或虚拟环境；
- V1 冻结标签保持不可变；
- 真机参数变化必须先通过语法、单元测试、dry-run 和小幅真机验证；
- 同一 CAN 总线只允许一个控制进程；
- 新功能不得以删除 Grip、XR stale 或关节限幅作为捷径。

Piper SDK 参考：[AgileX Robotics Piper SDK](https://github.com/agilexrobotics/piper_sdk)。仓库内 SDK 遵循其自身许可证。


## 17. LeRobot v2.1 数据集

仓库包含一个经过实际加载测试的 LeRobot v2.1 样例：

```text
piper_teleop/lerobot_datasets/session_20260827_211359/
```

样例来自三路摄像头和一份同步关节 CSV：

| 项目 | 值 |
|---|---|
| Episodes | 1 |
| Frames | 716 |
| FPS | 30 |
| Cameras | `camera_0`, `camera_2`, `camera_8` |
| Image size | 640 x 480 |
| Joint dimensions | 12 |
| Joint unit | radians |
| Dataset format | LeRobot v2.1 |

### 转换新的录制

录制目录需要包含 `joint_angles.csv` 和一个或多个 `camera_*.mp4`。CSV 每一行必须与每个视频的一帧对应。转换器默认使用 `frame_index / fps` 生成标准 `timestamp`，并把原始 CSV 时间戳保存为 `observation.source_timestamp`，这样可以保留原始录制中的时间间隔异常而不破坏视频同步。

安装转换依赖：

```bash
python -m pip install -r piper_teleop/requirements-dataset.txt
sudo apt install ffmpeg
```

执行转换：

```bash
python piper_teleop/scripts/convert_to_lerobot.py \
  --input /path/to/recordings/session_YYYYMMDD_HHMMSS \
  --output /path/to/lerobot_datasets/session_YYYYMMDD_HHMMSS \
  --angle-unit radians \
  --timestamp-mode frame \
  --video-mode transcode
```

转换器会检查 CSV 行数与所有视频帧数是否一致，并将视频转码为 H.264、写入 Parquet/JSONL 元数据和 episode 统计信息。`observation.state` 与 `action` 都是录制到的 12 维关节命令；当前采集没有独立的实测关节状态流，因此不会伪造两者之间的差异。

### 加载测试

LeRobot v2.1 数据集使用与之匹配的 LeRobot 版本加载。当前样例已用 `lerobot==0.3.3`、CPU PyTorch、PyAV 实际验证：

```python
from pathlib import Path
from lerobot.datasets.lerobot_dataset import LeRobotDataset

dataset = LeRobotDataset(
    repo_id="local/piper_session_20260827_211359",
    root=Path("piper_teleop/lerobot_datasets/session_20260827_211359"),
    revision="v2.1",
    download_videos=False,
    video_backend="pyav",
)

print(len(dataset))                 # 716
print(dataset.num_episodes)         # 1
print(dataset[0]["observation.state"].shape)  # torch.Size([12])
print(dataset[0]["observation.images.camera_0"].shape)  # [3, 480, 640]
```

样例的 `dataset[0]`、`dataset[213]` 和 `dataset[715]` 均已成功读取，三路视频均能解码。
