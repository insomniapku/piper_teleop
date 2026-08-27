# Teleoperation Versions

本文定义仓库中可以连接真机的版本边界。完整启动和排障方法以根目录 [README.md](README.md) 为准。

## V1 Cartesian Stable

- 状态：`SUPPORTED`，冻结回退版本；
- 发布分支：`release/piper-teleop-v1`；
- 快照分支：`snapshot/piper-teleop-v1-working-20260826`；
- 标签：`piper-teleop-v1-working-20260826`；
- 验证提交：`6b2b18b08d394118fcfdc473df25d957c9fe27b5`；
- 核心：`pico_teleop_piper_fixed.py`；
- 正式入口：`run_piper_normal_teleop.sh`；
- 恢复测试：`run_piper_safe_teleop.sh`。

V1 使用 Piper `EndPoseCtrl`。正式入口跟随位置并保持 Grip 激活时的末端姿态，不启用实验性欧拉角姿态映射。

## V2 Single-arm Joint IK

- 状态：`LIVE TESTED`；
- 分支：`feature/piper-ik-orientation-v2`；
- 核心：`pico_teleop_piper_ik_v2.py`、`piper_ik_v2.py`；
- 入口：`run_piper_v2_single.sh`；
- 映射：左手柄 → `can0` 左 Piper。

数据链：

```text
PICO relative position + quaternion
                 -> base-frame SE(3) target
                 -> Pinocchio position-priority / soft-orientation IK
                 -> continuous J1-J6 target
                 -> 5 deg/frame output gate
                 -> Piper JointCtrl
```

现场参数为位置比例 `0.8`、旋转比例 `1.0`、姿态范围 `±180°`、关节输出上限 `5°/frame`、IK 位置容差 `2 mm` 和 Piper 速度 `100%`。

## V2 Bimanual Joint IK

- 状态：`LIVE TESTED / EXPERIMENTAL`；
- 核心：`pico_teleop_piper_bimanual_v2.py`；
- 入口：`run_piper_v2_bimanual.sh`；
- 映射：左手柄 → `can0` 左臂，右手柄 → `can1` 右臂。

双臂版本只初始化一个 XR SDK 客户端，在同一 50 Hz 循环内管理两套完全独立的：

- Grip clutch 与重定参考；
- 真机关节反馈；
- Pinocchio IK 状态；
- IK HOLD 与 5°关节步长限制；
- Trigger/夹爪状态；
- CAN 输出。

现场已经观察到左右 `XR READY`、左右 Grip 激活、两路关节目标和同时操作。仍标记为实验性，因为尚未实现双臂碰撞检测、协同约束或统一奇异位形规划。

不要用两份单臂进程代替双臂入口。XRoboToolkit 服务在两个 Python 客户端并行时可能让第二客户端收到零位姿，并导致主客户端 `XR STALE`。

## V2 仍保留的保护

- Grip 激活/释放阈值为 `0.80/0.60`；
- 首个目标等于 Grip 激活时的真机测量关节；
- XR 超过 `0.2 s` 不更新即停止；
- XR 恢复后要求 10 个连续有效帧；
- IK 位置残差超过 `2 mm`、不可达或求解失败时保持上一命令；
- 每个关节每帧最多变化 `5°`；
- URDF 关节边界与 Piper 固件保护继续有效；
- 二值夹爪只允许完整张开/完整闭合。

`--no-speed-limit` 与 `--no-workspace-limit` 仅关闭应用层笛卡尔速度和 XYZ 裁剪，不代表机械臂没有任何保护。不要在真机上使用 `--no-joint-step-limit`；现场日志存在超过 50°、甚至 100°的原始 IK 请求跳变。

## Legacy / Experiments

以下文件保留用于追溯，不作为新的真机入口：

- `pico_teleop_piper.py`；
- `pico_teleop_improved.py`；
- 旧 PyKDL/Placo 求解器；
- `piper_teleop/` 下旧 ROS 节点和重复副本；
- SDK 运动演示脚本。

## 发布规则

1. V1 快照标签不可变。
2. 真机入口必须有根目录启动脚本和明确的状态标记。
3. 任何控制参数变化都必须通过 `py_compile`、单元测试、完整 dry-run 和小幅真机验证。
4. 不提交 `.venv/`、日志、录制数据、密码、令牌或主机凭据。
5. 同一 CAN 总线不得同时运行多个控制程序。
6. 双臂版本在具备碰撞检测前继续标为 `EXPERIMENTAL`。
