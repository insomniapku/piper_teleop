# Pico XR → Piper 快速修复报告

## 基线与入口

- 实际目标仓库：`https://github.com/gxccc123/piper_teleop.git`（PRIMARY HTTPS 克隆成功，未使用备用仓库）
- 修改前 commit：`69ee69349a4962652d1711769f1a0f8f4a6c813a`
- 修复分支：`fix/pico-teleop-latency`
- 当前直连 Pico/Piper 的实际入口：根目录 `pico_teleop_piper.py`（脚本自身说明直接运行，且直接调用 XR getter 与 Piper SDK）。README 另推荐 ROS/ROS2 launch，但包内 README 仍把 Pico 映射和硬件接口标为待实现；`piper_teleop/` 下还存在一套重复的 Python package 文件。

## 根因与最小修复

- 坐标系：旧直连脚本直接计算 `xr_pos - xr_ref_pos`，未应用官方 `R_HEADSET_TO_WORLD`。新脚本集中定义 XR→Piper 矩阵，并支持安装 yaw 0/90/-90/180°。
- MOVEL/MOVEP：旧脚本每帧重发 MOVEL 模式，易积累路径追赶。新脚本启动和 grip rising edge 确认 MOVEP，循环主要只发最新 `EndPoseCtrl`。
- `max_delta`：旧值 0.08 m 截断的是相对参考点的总位移，并非单周期步长。新脚本先裁剪工作空间，再按真实 monotonic `dt` 和 0.08 m/s 限速。
- 控制周期：旧脚本简单 `sleep(0.01)`，执行耗时叠加且实际为 100 Hz。新脚本使用默认 50 Hz deadline loop 和 0.005–0.05 s 的安全 `dt`。
- XR timestamp：旧脚本不检查数据是否更新。新脚本不从重复 timestamp 生成目标；超过 0.2 s 未更新即清参考并要求松开后重新 grip。
- 固定姿态：旧脚本始终写死 `(-179900, 0, -179900)`。新脚本在 grip rising edge 读取完整实际末端反馈，保持当时 RX/RY/RZ；反馈不完整时禁止发送运动目标。
- Grip/夹爪：加入 0.80/0.60 滞回；松开只停止目标，不回 home、不失能。保留 trigger 0=开、1=闭的旧方向，仅变化超过 0.01 或间隔达到 0.2 s 才刷新。
- 安全门：默认 dry-run。只有 `--hardware` 且交互输入 `ARM` 后才导入硬件 SDK、连接/使能并允许命令；退出不回 home、不 Reset、不 Disable。

修改文件：`pico_teleop_piper_fixed.py`、`CODEX_FIX_REPORT.md`。旧脚本未改、未删。

## 无硬件检查

- `python -m py_compile pico_teleop_piper_fixed.py`：通过。
- `python pico_teleop_piper_fixed.py --dry-run`：通过；映射非 identity、第一帧无跳变、速度限制触发、值均 finite、目标更新 50.0 Hz、0 overrun。
- 本次没有初始化 XR、连接 CAN、实例化 Piper interface 或执行任何机器人/夹爪命令。

## 真实机器人测试（本次未执行）

首次 commissioning：

```bash
python pico_teleop_piper_fixed.py \
    --hardware \
    --safe-test \
    --can-name can0 \
    --yaw-deg 0
```

确认三轴后，若安装方向不符，保持 `--safe-test`，依次尝试 `--yaw-deg 90`、`--yaw-deg -90`、`--yaw-deg 180`；每次均需先松开 grip，并确保工作空间安全。正常参数：

```bash
python pico_teleop_piper_fixed.py \
    --hardware \
    --can-name can0 \
    --position-scale 1.0 \
    --max-speed 0.08 \
    --speed-percent 40 \
    --yaw-deg 0
```

回退方式：停止新脚本即可；旧 `pico_teleop_piper.py` 和 `pico_teleop_improved.py` 均保留原样。注意旧脚本启动会自动运动到固定 home，不能把它当作安全回退命令直接在未知现场运行。

仍需硬件确认：实际 Pico 安装 yaw 与三轴方向；固件在 MOVEP 下对连续最新点的覆盖/跟随表现；0.08 m/s、工作空间及末端反馈频率是否适合现场负载。重点观察手柄停止后的追赶量，以及每次重新握住 grip 是否完全无跳变。
