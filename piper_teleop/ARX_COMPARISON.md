# ARX 对照与采集卡顿修复

对照版本：Piper `c2603de4990cc6c52ca0c07072c5fe82f099d5a2`，
XRoboToolkit Python 示例 `79e5cb8a56e3455515ce1b476e993c764ec58739`。
本次从已发布工作版本检查；尚未读取 chongyu 的未提交改动，也未进行真机测试。

| 项目 | 官方 ARX 示例 | Piper 原工作版本 | 本次处理 |
|---|---|---|---|
| IK / 命令发送 | IK 和硬件发送分别运行于线程，默认 50 Hz | 同一循环依次解左右 IK 并发送，录制也依赖其完成次数 | 独立反馈采样和录制时钟；记录控制耗时，电机发送仍由原控制路径负责 |
| 开始 / 保存录制 | 独立日志线程处理 B 键与保存 | 主控制循环直接初始化相机、打开编码器、等待队列排空和关闭文件 | 按键只发请求，录制管理线程处理慢 I/O |
| 反馈采样 | 日志线程读取 qpos / qvel，与 qpos_des 分开 | 每个 IK 控制周期取状态；反馈失败可回退 previous_joints 并使用当前时间 | 独立标称 60 Hz 采真实反馈；缺失、过期或两臂时间差过大时计数并跳过 |
| 图像 | 相机线程和压缩帧缓存；ARX 默认 424×240、60 fps，日志默认 50 Hz | 三路 640×480、30 fps，独立读取后串行编码到 MP4 | 保留现有相机设置；编码积压时限制请求队列至 2，丢弃过期请求并报告 |
| 同步 | 记录读取到的机器人状态和缓存图像；并非曝光级同步 | 最近帧匹配、相机容差 20 ms、状态容差 12 ms | 检查多个可配对参考帧，避免卡在首个无法匹配的参考帧；保留容差 |
| 诊断 | 示例不提供完整端到端时延证明 | 掉帧总数，缺少有效 FPS / 实际间隔 / IK 循环耗时 | 增加实际采集间隔、有效 FPS、积压延迟、控制超时、反馈采样统计和离线诊断工具 |
| 夹爪 / 数据结构 | 连续夹爪目标；qpos、qvel、qpos_des、gripper_target | 启动脚本为二值夹爪；CSV 有夹爪控制状态，转换器仅导出 12 个关节且 state/action 同源 | 本次优先修复采集时序；训练字段语义仍需单独迁移与兼容验证 |
| IK 算法 | Placo 增量约束求解 | Pinocchio 数值 IK，每臂最多 80 次迭代，软姿态接受 | 本次未改 IK 数学、关节限幅、Grip 或 XR 超时；控制统计用于判断后续是否需要替换求解器 |

主要源码：

- [ARX 控制与日志线程](https://github.com/XR-Robotics/XRoboToolkit-Teleop-Sample-Python/blob/79e5cb8a56e3455515ce1b476e993c764ec58739/xrobotoolkit_teleop/common/base_hardware_teleop_controller.py)
- [ARX 默认频率与状态字段](https://github.com/XR-Robotics/XRoboToolkit-Teleop-Sample-Python/blob/79e5cb8a56e3455515ce1b476e993c764ec58739/xrobotoolkit_teleop/hardware/arx_r5_teleop_controller.py)
- [Piper 原控制循环](https://github.com/insomniapku/piper_teleop/blob/c2603de4990cc6c52ca0c07072c5fe82f099d5a2/piper_teleop/pico_teleop_piper_bimanual_v2.py)
- [Piper 原录制器](https://github.com/insomniapku/piper_teleop/blob/c2603de4990cc6c52ca0c07072c5fe82f099d5a2/piper_teleop/trajectory_recorder.py)

## 验收

离线测试只使用假反馈、假相机和假写盘，不导入 XR / Piper SDK 或连接 CAN。
覆盖主线程停顿时持续采样、慢相机初始化时立即响应停止请求、缺失/过期/错时
反馈拒绝、编码器积压丢弃旧请求、跨相机可配对参考帧选择及 CSV/视频帧数一致。

现场应在相同任务和相机条件下采旧版与新版各一段短轨迹，再运行
`scripts/analyze_recording_timing.py <session>`。检查有效 FPS、长间隔、
控制超时和反馈间隔，同时播放原始三路视频。独立采样能消除代码中的时序耦合，
不能单凭离线测试排除 USB 带宽、相机驱动、编码性能、XR 断流或机械臂自身停顿。
固定 FPS 视频中的长间隔仍须显式处理后再用于训练。
