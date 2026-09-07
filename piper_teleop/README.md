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

录制时控制频率为 60 Hz、视频为 30 fps。摄像头由独立线程持续采集，
机械臂实际反馈在每个控制周期采样，写盘线程按时间戳匹配最近的状态和视频帧。
`joint_angles.csv` 同时保存 `robot_state_timestamp`、各臂反馈时间、
各臂夹爪控制状态（`left_gripper_trigger`、`right_gripper_trigger`，
`0.0` 为打开、`1.0` 为闭合）、各路相机时间及匹配误差；
`sync_report.json` 保存本次录制的丢帧和阈值统计。
