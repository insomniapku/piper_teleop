# LeRobot v2.1 Dataset

- Robot: `piper_bimanual`
- Task: `bimanual Piper teleoperation`
- Episodes: 1
- Frames: 716
- FPS: 30
- Joint angle unit: `radians`
- Cameras: observation.images.camera_0, observation.images.camera_2, observation.images.camera_8

`observation.state` and `action` both contain the recorded 12-joint command
because the source recording has no separate measured joint-state stream.
Video and CSV rows are synchronized by frame index. The original CSV
timestamps are available in `observation.source_timestamp`; the standard
`timestamp` column uses `frame` mode.
