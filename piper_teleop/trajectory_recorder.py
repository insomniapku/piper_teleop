#!/usr/bin/env python3
"""Trajectory recorder for bimanual teleoperation with synchronized video.

Records:
- Left arm joint angles (6 DOF)
- Right arm joint angles (6 DOF)
- Three camera feeds (video0, video2, video4)
- Timestamps for synchronization
"""

import csv
import json
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from queue import Empty, Full, Queue
from typing import Optional

import cv2
import numpy as np


@dataclass(frozen=True)
class CameraFrame:
    """A decoded camera frame with its host-side capture timestamp."""

    timestamp_ns: int
    sequence: int
    image: np.ndarray


@dataclass(frozen=True)
class RobotStateSample:
    """Actual robot feedback sampled on the host monotonic clock."""

    timestamp_ns: int
    sequence: int
    left_joints_rad: np.ndarray
    right_joints_rad: np.ndarray
    left_timestamp_ns: int
    right_timestamp_ns: int
    left_host_timestamp_ns: int
    right_host_timestamp_ns: int
    left_device_timestamp: Optional[float]
    right_device_timestamp: Optional[float]


@dataclass(frozen=True)
class RecordingRequest:
    """Request to associate one video bundle with the nearest robot state."""

    timestamp_ns: int


@dataclass
class RecordingSession:
    """Single recording session metadata and file handles."""
    session_dir: Path
    start_time_ns: int
    joint_csv_path: Path
    video_paths: dict[int, Path] = field(default_factory=dict)
    csv_writer: Optional[csv.writer] = None
    csv_file: Optional[object] = None
    video_writers: dict[int, cv2.VideoWriter] = field(default_factory=dict)
    frame_count: int = 0
    dropped_bundle_count: int = 0
    queue_full_count: int = 0
    sync_timeout_count: int = 0
    writer_error_count: int = 0

    def close(self):
        """Close all file handles."""
        if self.csv_file:
            self.csv_file.close()
        for writer in self.video_writers.values():
            if writer:
                writer.release()


class TrajectoryRecorder:
    """Records joint trajectories and synchronized camera feeds."""

    def __init__(
        self,
        camera_indices: tuple[int, int, int] = (0, 2, 4),
        fps: int = 30,
        recording_dir: str = "recordings",
    ):
        self.camera_indices = camera_indices
        self.fps = fps
        self.recording_dir = Path(recording_dir)
        self.recording_dir.mkdir(exist_ok=True)

        self.is_recording = False
        self.session: Optional[RecordingSession] = None
        self.lock = threading.Lock()

        # Camera capture objects
        self.captures: dict[int, cv2.VideoCapture] = {}
        self.camera_threads: dict[int, threading.Thread] = {}
        self.camera_stop_flags: dict[int, threading.Event] = {}
        self.camera_buffers: dict[int, deque[CameraFrame]] = {}
        self.camera_sequences: dict[int, int] = {}
        self.last_written_sequences: dict[int, int] = {}
        self.camera_condition = threading.Condition()
        self.state_buffer: deque[RobotStateSample] = deque(maxlen=128)
        self.state_sequence = 0
        self.last_written_state_sequence = 0

        # Camera capture and video encoding must never run in the 60 Hz control thread.
        self.record_queue: Queue[Optional[RecordingRequest]] = Queue(maxsize=16)
        self.writer_thread: Optional[threading.Thread] = None
        self.camera_wait_timeout_ns = 50_000_000
        self.max_camera_skew_ns = 20_000_000
        self.max_state_skew_ns = 12_000_000

    def _init_cameras(self) -> bool:
        """Initialize camera captures. Returns True even if no cameras available."""
        import os
        import grp

        # Check if user has video group permission
        try:
            video_gid = grp.getgrnam('video').gr_gid
            user_groups = os.getgroups()
            if video_gid not in user_groups:
                print(f"[RECORDER] Warning: User not in 'video' group. Add with: sudo usermod -aG video $USER")
                print(f"[RECORDER] Camera recording will be skipped. Joint angles will still be recorded.")
                return True  # Continue without cameras
        except KeyError:
            pass

        for idx in self.camera_indices:
            try:
                cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
                if not cap.isOpened():
                    print(f"[RECORDER] Warning: camera {idx} failed to open")
                    continue

                # Set resolution and FPS
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                cap.set(cv2.CAP_PROP_FPS, self.fps)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

                self.captures[idx] = cap
                self.camera_buffers[idx] = deque(maxlen=12)
                self.camera_sequences[idx] = 0
                print(f"[RECORDER] Camera {idx} initialized: "
                      f"{int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
                      f"{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))} @ "
                      f"{int(cap.get(cv2.CAP_PROP_FPS))}fps")
            except Exception as e:
                print(f"[RECORDER] Error initializing camera {idx}: {e}")
                continue

        if len(self.captures) == 0:
            print(f"[RECORDER] No cameras available. Recording only joint angles.")

        self._start_camera_threads()

        return True  # Always return True to allow joint-only recording

    def _start_camera_threads(self) -> None:
        for cam_idx, cap in self.captures.items():
            if cam_idx in self.camera_threads:
                continue
            stop_flag = threading.Event()
            thread = threading.Thread(
                target=self._camera_loop,
                args=(cam_idx, cap, stop_flag),
                name=f"piper-camera-{cam_idx}",
                daemon=True,
            )
            self.camera_stop_flags[cam_idx] = stop_flag
            self.camera_threads[cam_idx] = thread
            thread.start()

    def _camera_loop(
        self,
        cam_idx: int,
        cap: cv2.VideoCapture,
        stop_flag: threading.Event,
    ) -> None:
        """Continuously capture frames without blocking the control loop."""
        while not stop_flag.is_set():
            read_start_ns = time.monotonic_ns()
            try:
                ret, frame = cap.read()
            except Exception:
                time.sleep(0.005)
                continue
            read_end_ns = time.monotonic_ns()
            timestamp_ns = (read_start_ns + read_end_ns) // 2
            if not ret:
                continue

            with self.camera_condition:
                sequence = self.camera_sequences[cam_idx] + 1
                self.camera_sequences[cam_idx] = sequence
                self.camera_buffers[cam_idx].append(
                    CameraFrame(timestamp_ns, sequence, frame)
                )
                self.camera_condition.notify_all()

    def _find_camera_frame_locked(
        self, cam_idx: int, target_timestamp_ns: int
    ) -> Optional[CameraFrame]:
        candidates = [
            frame
            for frame in self.camera_buffers[cam_idx]
            if frame.sequence > self.last_written_sequences.get(cam_idx, 0)
        ]
        if not candidates:
            return None
        return min(
            candidates,
            key=lambda frame: abs(frame.timestamp_ns - target_timestamp_ns),
        )

    def _find_state_locked(
        self, target_timestamp_ns: int
    ) -> Optional[RobotStateSample]:
        candidates = [
            sample
            for sample in self.state_buffer
            if sample.sequence > self.last_written_state_sequence
        ]
        if not candidates:
            return None
        return min(
            candidates,
            key=lambda sample: abs(sample.timestamp_ns - target_timestamp_ns),
        )

    def record_state(
        self,
        left_joints_rad: np.ndarray,
        right_joints_rad: np.ndarray,
        left_timestamp_ns: int,
        right_timestamp_ns: int,
        left_device_timestamp: Optional[float] = None,
        right_device_timestamp: Optional[float] = None,
        left_host_timestamp_ns: Optional[int] = None,
        right_host_timestamp_ns: Optional[int] = None,
    ) -> None:
        """Store one actual-feedback sample without blocking the control loop."""
        timestamp_ns = max(left_timestamp_ns, right_timestamp_ns)
        if left_host_timestamp_ns is None:
            left_host_timestamp_ns = left_timestamp_ns
        if right_host_timestamp_ns is None:
            right_host_timestamp_ns = right_timestamp_ns
        with self.camera_condition:
            self.state_sequence += 1
            self.state_buffer.append(
                RobotStateSample(
                    timestamp_ns=timestamp_ns,
                    sequence=self.state_sequence,
                    left_joints_rad=np.asarray(left_joints_rad, dtype=np.float64).copy(),
                    right_joints_rad=np.asarray(right_joints_rad, dtype=np.float64).copy(),
                    left_timestamp_ns=left_timestamp_ns,
                    right_timestamp_ns=right_timestamp_ns,
                    left_host_timestamp_ns=left_host_timestamp_ns,
                    right_host_timestamp_ns=right_host_timestamp_ns,
                    left_device_timestamp=left_device_timestamp,
                    right_device_timestamp=right_device_timestamp,
                )
            )
            self.camera_condition.notify_all()

    def _wait_for_synchronized_bundle(
        self, target_timestamp_ns: int, camera_ids: list[int]
    ) -> Optional[tuple[RobotStateSample, dict[int, CameraFrame], int]]:
        """Match actual state to new camera frames without interpolation."""
        deadline_ns = time.monotonic_ns() + self.camera_wait_timeout_ns
        with self.camera_condition:
            while True:
                if camera_ids:
                    reference = self._find_camera_frame_locked(
                        camera_ids[0], target_timestamp_ns
                    )
                    if reference is None:
                        reference_timestamp_ns = target_timestamp_ns
                        selected: dict[int, Optional[CameraFrame]] = {}
                    else:
                        reference_timestamp_ns = reference.timestamp_ns
                        selected = {
                            camera_ids[0]: reference,
                            **{
                                cam_idx: self._find_camera_frame_locked(
                                    cam_idx, reference_timestamp_ns
                                )
                                for cam_idx in camera_ids[1:]
                            },
                        }
                else:
                    reference_timestamp_ns = target_timestamp_ns
                    selected = {}

                state = self._find_state_locked(reference_timestamp_ns)
                frames_ready = len(selected) == len(camera_ids) and all(
                    frame is not None for frame in selected.values()
                )
                if state is not None and frames_ready:
                    typed = {
                        cam_idx: frame
                        for cam_idx, frame in selected.items()
                        if frame is not None
                    }
                    state_error = abs(
                        state.timestamp_ns - reference_timestamp_ns
                    )
                    camera_errors = [
                        abs(frame.timestamp_ns - reference_timestamp_ns)
                        for frame in typed.values()
                    ]
                    if (
                        state_error <= self.max_state_skew_ns
                        and max(camera_errors, default=0) <= self.max_camera_skew_ns
                    ):
                        for cam_idx, frame in typed.items():
                            self.last_written_sequences[cam_idx] = frame.sequence
                        self.last_written_state_sequence = state.sequence
                        return state, typed, reference_timestamp_ns

                remaining_ns = deadline_ns - time.monotonic_ns()
                if remaining_ns <= 0:
                    return None
                self.camera_condition.wait(timeout=remaining_ns / 1.0e9)

    def _writer_loop(self, session: RecordingSession) -> None:
        camera_ids = sorted(session.video_writers)
        while True:
            try:
                bundle = self.record_queue.get()
            except Empty:
                continue

            try:
                if bundle is None:
                    return

                synchronized = self._wait_for_synchronized_bundle(
                    bundle.timestamp_ns, camera_ids
                )
                if synchronized is None:
                    session.dropped_bundle_count += 1
                    session.sync_timeout_count += 1
                    continue
                state, camera_frames, reference_timestamp_ns = synchronized

                for cam_idx in camera_ids:
                    session.video_writers[cam_idx].write(camera_frames[cam_idx].image)

                timestamp = (
                    reference_timestamp_ns - session.start_time_ns
                ) / 1.0e9
                state_timestamp = (
                    state.timestamp_ns - session.start_time_ns
                ) / 1.0e9
                state_camera_delta = (
                    reference_timestamp_ns - state.timestamp_ns
                ) / 1.0e9
                left_state_timestamp = (
                    state.left_timestamp_ns - session.start_time_ns
                ) / 1.0e9
                right_state_timestamp = (
                    state.right_timestamp_ns - session.start_time_ns
                ) / 1.0e9
                left_state_host_timestamp = (
                    state.left_host_timestamp_ns - session.start_time_ns
                ) / 1.0e9
                right_state_host_timestamp = (
                    state.right_host_timestamp_ns - session.start_time_ns
                ) / 1.0e9
                left_deg = np.rad2deg(state.left_joints_rad)
                right_deg = np.rad2deg(state.right_joints_rad)
                row = [
                    f"{timestamp:.6f}",
                    *[f"{angle:.6f}" for angle in left_deg],
                    *[f"{angle:.6f}" for angle in right_deg],
                    f"{state_timestamp:.6f}",
                    f"{state_camera_delta:.6f}",
                    f"{left_state_timestamp:.6f}",
                    f"{right_state_timestamp:.6f}",
                    f"{left_state_host_timestamp:.6f}",
                    f"{right_state_host_timestamp:.6f}",
                    (
                        ""
                        if state.left_device_timestamp is None
                        else f"{state.left_device_timestamp:.6f}"
                    ),
                    (
                        ""
                        if state.right_device_timestamp is None
                        else f"{state.right_device_timestamp:.6f}"
                    ),
                ]
                for cam_idx in camera_ids:
                    camera_timestamp = (
                        camera_frames[cam_idx].timestamp_ns - session.start_time_ns
                    ) / 1.0e9
                    camera_delta = (
                        camera_frames[cam_idx].timestamp_ns - state.timestamp_ns
                    ) / 1.0e9
                    row.extend([f"{camera_timestamp:.6f}", f"{camera_delta:.6f}"])

                session.csv_writer.writerow(row)
                session.csv_file.flush()
                session.frame_count += 1
            except Exception as exc:
                session.dropped_bundle_count += 1
                session.writer_error_count += 1
                print(f"[RECORDER] Writer dropped bundle: {exc}")
            finally:
                self.record_queue.task_done()

    def start_recording(self) -> bool:
        """Start a new recording session."""
        with self.lock:
            if self.is_recording:
                print("[RECORDER] Already recording")
                return False

            # Initialize cameras if not done
            if not self.captures and not hasattr(self, '_camera_init_attempted'):
                self._camera_init_attempted = True
                self._init_cameras()

            # Create session directory
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            session_dir = self.recording_dir / f"session_{timestamp}"
            session_dir.mkdir(parents=True, exist_ok=True)

            # Create CSV file for joint angles
            joint_csv_path = session_dir / "joint_angles.csv"
            csv_file = open(joint_csv_path, 'w', newline='')
            csv_writer = csv.writer(csv_file)
            header = [
                'timestamp',
                'left_j1', 'left_j2', 'left_j3', 'left_j4', 'left_j5', 'left_j6',
                'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6',
                'robot_state_timestamp',
                'state_camera_delta_seconds',
                'left_state_timestamp',
                'right_state_timestamp',
                'left_state_host_read_timestamp',
                'right_state_host_read_timestamp',
                'left_feedback_device_timestamp',
                'right_feedback_device_timestamp',
            ]
            for cam_idx in sorted(self.captures):
                header.extend([
                    f'camera_{cam_idx}_timestamp',
                    f'camera_{cam_idx}_delta_seconds',
                ])
            csv_writer.writerow(header)

            # Create video writers for each camera
            video_writers = {}
            video_paths = {}
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')

            for cam_idx in self.captures.keys():
                try:
                    cap = self.captures[cam_idx]
                    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    video_path = session_dir / f"camera_{cam_idx}.mp4"
                    writer = cv2.VideoWriter(
                        str(video_path), fourcc, self.fps, (width, height)
                    )
                    if not writer.isOpened():
                        print(f"[RECORDER] Failed to create video writer for camera {cam_idx}")
                        continue

                    video_writers[cam_idx] = writer
                    video_paths[cam_idx] = video_path
                except Exception as e:
                    print(f"[RECORDER] Error creating video writer for camera {cam_idx}: {e}")
                    continue

            # Create session object
            self.session = RecordingSession(
                session_dir=session_dir,
                start_time_ns=time.monotonic_ns(),
                joint_csv_path=joint_csv_path,
                video_paths=video_paths,
                csv_writer=csv_writer,
                csv_file=csv_file,
                video_writers=video_writers,
            )

            with self.camera_condition:
                self.last_written_sequences = dict(self.camera_sequences)
                self.last_written_state_sequence = self.state_sequence
                self.state_buffer.clear()

            self.writer_thread = threading.Thread(
                target=self._writer_loop,
                args=(self.session,),
                name="piper-recording-writer",
                daemon=True,
            )
            self.writer_thread.start()

            self.is_recording = True
            print(f"[RECORDER] Started recording to {session_dir}")
            if video_writers:
                print(f"[RECORDER] Recording {len(video_writers)} camera(s): "
                      f"{list(video_writers.keys())}")
            else:
                print(f"[RECORDER] Recording joint angles only (no cameras)")
            return True

    def record_frame(
        self,
        timestamp_ns: Optional[int] = None,
    ):
        """Queue one state/frame bundle without blocking the control loop."""
        with self.lock:
            if not self.is_recording or not self.session:
                return

            if timestamp_ns is None:
                timestamp_ns = time.monotonic_ns()
            bundle = RecordingRequest(timestamp_ns=timestamp_ns)
            try:
                self.record_queue.put_nowait(bundle)
            except Full:
                self.session.dropped_bundle_count += 1
                self.session.queue_full_count += 1

    def stop_recording(self) -> Optional[Path]:
        """Stop the current recording session."""
        with self.lock:
            if not self.is_recording or not self.session:
                print("[RECORDER] Not currently recording")
                return None

            session_dir = self.session.session_dir
            self.is_recording = False

        # Drain all queued bundles before closing video and CSV handles.
        self.record_queue.join()
        if self.writer_thread:
            self.record_queue.put(None)
            self.record_queue.join()
            self.writer_thread.join(timeout=2.0)
            self.writer_thread = None

        with self.lock:
            session = self.session
            self.session = None
        if session:
            session.close()
            frame_count = session.frame_count
            dropped_count = session.dropped_bundle_count
            sync_report = {
                "format": "piper_trajectory_recording_v3",
                "fps": self.fps,
                "frames_written": frame_count,
                "dropped_bundles": dropped_count,
                "queue_full_count": session.queue_full_count,
                "sync_timeout_count": session.sync_timeout_count,
                "writer_error_count": session.writer_error_count,
                "camera_ids": sorted(session.video_writers),
                "uses_interpolation": False,
                "camera_timestamp_source": (
                    "host monotonic midpoint around cv2.VideoCapture.read()"
                ),
                "robot_state_timestamp_source": (
                    "Piper CAN feedback timestamp mapped to host monotonic clock"
                ),
                "host_state_read_timestamp_source": "host monotonic clock after GetArmJointMsgs()",
                "max_camera_skew_ms": self.max_camera_skew_ns / 1.0e6,
                "max_state_skew_ms": self.max_state_skew_ns / 1.0e6,
            }
            (session_dir / "sync_report.json").write_text(
                json.dumps(sync_report, indent=2) + "\n",
                encoding="utf-8",
            )
        else:
            frame_count = 0
            dropped_count = 0

        print(
            f"[RECORDER] Stopped recording: {frame_count} frames saved to {session_dir}"
            f"; dropped bundles={dropped_count}"
        )
        return session_dir

    def cleanup(self):
        """Release all camera resources."""
        if self.is_recording:
            self.stop_recording()

        for stop_flag in self.camera_stop_flags.values():
            stop_flag.set()
        for cap in self.captures.values():
            cap.release()
        for thread in self.camera_threads.values():
            thread.join(timeout=1.0)
        self.captures.clear()
        self.camera_threads.clear()
        self.camera_stop_flags.clear()
        print("[RECORDER] Cleanup complete")
