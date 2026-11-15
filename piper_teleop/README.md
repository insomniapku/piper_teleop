# Piper Teleoperation ROS 2 Package

这是仓库中的 ROS 2 `ament_python` 包，提供 Pico XR 输入接口、Piper 遥操作节点、数据采集节点、启动文件和离线测试。

完整的硬件安全说明、单臂/双臂 V2 入口以及 LeRobot 数据集转换说明请查看仓库根目录的 [README.md](../README.md)。

## 主要入口

```text
pico_teleop_piper_ik_v2.py       # 单臂 V2 四元数姿态 IK
pico_teleop_piper_bimanual_v2.py # 单 XR 客户端双臂 V2
trajectory_recorder.py           # 关节 CSV + 三路视频同步录制
scripts/convert_to_lerobot.py    # CSV/MP4 -> LeRobot v2.1
```

## ROS 2 构建

```bash
cd /path/to/workspace
source /opt/ros/jazzy/setup.bash
colcon build --packages-select piper_teleop
source install/setup.bash
```

## ROS 2 启动

使用 mock XR 输入进行数据采集测试：

```bash
ros2 launch piper_teleop data_collection.launch.py \
  xr_mode:=mock \
  record_camera:=false
```

实际 Pico XR 数据采集和真机遥操作前，请先阅读根目录 README 中的 CAN、XR 连接、Grip 离合器和急停要求。

## 数据集依赖

```bash
python -m pip install -r requirements-dataset.txt
```

转换脚本默认生成 LeRobot v2.1 格式，并已用本仓库中的三摄像头样例完成 `LeRobotDataset` 加载测试。

录制入口为本目录 `run_piper_v2_bimanual_with_recording.sh`。反馈采样标称
60 Hz、视频标称 30 fps；反馈与录制请求使用独立定时线程，不再依赖双臂
IK 循环完成次数。相机初始化、B 键开始/停止录制及文件收尾由独立管理线程处理。
摄像头持续采集，写盘线程按时间戳匹配最近的实测反馈和视频帧。
`joint_angles.csv` 同时保存 `robot_state_timestamp`、各臂反馈时间、
各臂夹爪控制状态（`left_gripper_trigger`、`right_gripper_trigger`，
`0.0` 为打开、`1.0` 为闭合）、各路相机时间及匹配误差；
`sync_report.json` 保存本次录制的丢帧和阈值统计。

反馈缺失、过期或左右时间差过大时跳过该反馈，不用关节命令冒充实测值。
编码器积压时只保留最近两个待处理请求，丢弃过期请求并计数；不通过复制视频
帧或插值掩盖缺失数据。CSV 额外保存帧索引、实际帧间隔和写盘延迟；报告包含
有效采集 FPS、长间隔、反馈采样间隔及控制循环工作耗时。相机时间仍为主机
读取区间的中点估计，不能视为硬件触发同步。

检查一段已有录制，不打开相机或机械臂：

```bash
python scripts/analyze_recording_timing.py recordings/session_YYYYMMDD_HHMMSS_SUFFIX
python -m unittest test_recording_pipeline.py -v
```

采集间隔超过标称周期的 1.5 倍会被诊断为长间隔。普通 MP4 仍为固定 FPS；
存在丢帧的录像可能压缩真实时间，转换前应检查原始时间戳，不能用更换时间戳
或重复图像来证明采集顺畅。ARX 源码对照与验收范围见
[ARX_COMPARISON.md](ARX_COMPARISON.md)。
