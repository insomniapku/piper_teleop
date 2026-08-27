#!/usr/bin/env python3
"""Trajectory recorder for bimanual teleoperation with synchronized video.

Records:
- Left arm joint angles (6 DOF)
- Right arm joint angles (6 DOF)
- Three camera feeds (video0, video2, video4)
- Timestamps for synchronization
"""

import csv
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import cv2
import numpy as np


@dataclass
class RecordingSession:
    """Single recording session metadata and file handles."""
    session_dir: Path
    start_time: float
    joint_csv_path: Path
    video_paths: dict[int, Path] = field(default_factory=dict)
    csv_writer: Optional[csv.writer] = None
    csv_file: Optional[object] = None
    video_writers: dict[int, cv2.VideoWriter] = field(default_factory=dict)
    frame_count: int = 0

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

                self.captures[idx] = cap
                print(f"[RECORDER] Camera {idx} initialized: "
                      f"{int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
                      f"{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))} @ "
                      f"{int(cap.get(cv2.CAP_PROP_FPS))}fps")
            except Exception as e:
                print(f"[RECORDER] Error initializing camera {idx}: {e}")
                continue

        if len(self.captures) == 0:
            print(f"[RECORDER] No cameras available. Recording only joint angles.")

        return True  # Always return True to allow joint-only recording

    def _camera_capture_loop(self, cam_idx: int):
        """Background thread for capturing camera frames."""
        cap = self.captures[cam_idx]
        writer = self.session.video_writers[cam_idx]
        stop_flag = self.camera_stop_flags[cam_idx]

        while not stop_flag.is_set():
            ret, frame = cap.read()
            if not ret:
                print(f"[RECORDER] Camera {cam_idx} read failed")
                time.sleep(0.001)
                continue

            with self.lock:
                if self.is_recording and writer:
                    writer.write(frame)

            time.sleep(1.0 / self.fps)

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
            csv_writer.writerow([
                'timestamp',
                'left_j1', 'left_j2', 'left_j3', 'left_j4', 'left_j5', 'left_j6',
                'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6',
            ])

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
                start_time=time.time(),
                joint_csv_path=joint_csv_path,
                video_paths=video_paths,
                csv_writer=csv_writer,
                csv_file=csv_file,
                video_writers=video_writers,
            )

            # Start camera capture threads
            for cam_idx in self.captures.keys():
                if cam_idx in video_writers:
                    stop_flag = threading.Event()
                    self.camera_stop_flags[cam_idx] = stop_flag
                    thread = threading.Thread(
                        target=self._camera_capture_loop,
                        args=(cam_idx,),
                        daemon=True,
                    )
                    thread.start()
                    self.camera_threads[cam_idx] = thread

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
        left_joints_rad: np.ndarray,
        right_joints_rad: np.ndarray,
    ):
        """Record a frame of joint angles."""
        with self.lock:
            if not self.is_recording or not self.session:
                return

            timestamp = time.time() - self.session.start_time
            left_deg = np.rad2deg(left_joints_rad)
            right_deg = np.rad2deg(right_joints_rad)

            self.session.csv_writer.writerow([
                f"{timestamp:.6f}",
                *[f"{angle:.6f}" for angle in left_deg],
                *[f"{angle:.6f}" for angle in right_deg],
            ])
            self.session.frame_count += 1

    def stop_recording(self) -> Optional[Path]:
        """Stop the current recording session."""
        with self.lock:
            if not self.is_recording or not self.session:
                print("[RECORDER] Not currently recording")
                return None

            # Stop camera threads
            for stop_flag in self.camera_stop_flags.values():
                stop_flag.set()

            for thread in self.camera_threads.values():
                thread.join(timeout=2.0)

            self.camera_threads.clear()
            self.camera_stop_flags.clear()

            # Close all files
            session_dir = self.session.session_dir
            frame_count = self.session.frame_count
            self.session.close()

            self.is_recording = False
            self.session = None

            print(f"[RECORDER] Stopped recording: {frame_count} frames saved to {session_dir}")
            return session_dir

    def cleanup(self):
        """Release all camera resources."""
        if self.is_recording:
            self.stop_recording()

        for cap in self.captures.values():
            cap.release()
        self.captures.clear()
        print("[RECORDER] Cleanup complete")
