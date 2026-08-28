#!/usr/bin/env python3
"""Convert a Piper teleoperation recording to a LeRobot v2.1 dataset.

The recorder writes one CSV row and one frame to each camera video together.
This converter uses that frame order as the synchronization source. The
standard LeRobot ``timestamp`` column is therefore generated from
``frame_index / fps`` by default, while the original CSV timestamp is kept as
``observation.source_timestamp``.
"""

from __future__ import annotations

import argparse
import csv
import json
import shutil
import subprocess
from fractions import Fraction
from pathlib import Path
from typing import Any

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq


JOINT_COLUMNS = [
    "left_j1",
    "left_j2",
    "left_j3",
    "left_j4",
    "left_j5",
    "left_j6",
    "right_j1",
    "right_j2",
    "right_j3",
    "right_j4",
    "right_j5",
    "right_j6",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert synchronized Piper CSV + MP4 recordings to LeRobot v2.1."
    )
    parser.add_argument(
        "--input",
        type=Path,
        required=True,
        help="Recording directory containing joint_angles.csv and camera_*.mp4.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        required=True,
        help="Destination directory for the LeRobot dataset.",
    )
    parser.add_argument(
        "--task",
        default="bimanual Piper teleoperation",
        help="Task text written to tasks.jsonl.",
    )
    parser.add_argument(
        "--robot-type",
        default="piper_bimanual",
        help="Value written to info.json robot_type.",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=None,
        help="Dataset FPS. Defaults to the common source video FPS.",
    )
    parser.add_argument(
        "--angle-unit",
        choices=("degrees", "radians"),
        default="radians",
        help="Unit stored in observation.state and action. Default: radians.",
    )
    parser.add_argument(
        "--timestamp-mode",
        choices=("frame", "raw"),
        default="frame",
        help="Use frame_index/fps or normalized source CSV timestamps.",
    )
    parser.add_argument(
        "--video-mode",
        choices=("transcode", "copy"),
        default="transcode",
        help="Transcode videos to H.264 avc1 or copy the original MP4 streams.",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Replace an existing destination directory.",
    )
    return parser.parse_args()


def run_command(command: list[str]) -> None:
    try:
        subprocess.run(command, check=True)
    except FileNotFoundError as exc:
        raise RuntimeError(
            f"Required command is not installed: {command[0]}"
        ) from exc
    except subprocess.CalledProcessError as exc:
        rendered = " ".join(command)
        raise RuntimeError(f"Command failed with exit code {exc.returncode}: {rendered}") from exc


def probe_video(path: Path) -> dict[str, Any]:
    command = [
        "ffprobe",
        "-v",
        "error",
        "-select_streams",
        "v:0",
        "-show_entries",
        "stream=codec_name,width,height,pix_fmt,r_frame_rate,avg_frame_rate,nb_frames,duration",
        "-of",
        "json",
        str(path),
    ]
    try:
        completed = subprocess.run(
            command,
            check=True,
            capture_output=True,
            text=True,
        )
    except FileNotFoundError as exc:
        raise RuntimeError("Required command is not installed: ffprobe") from exc
    except subprocess.CalledProcessError as exc:
        raise RuntimeError(f"Unable to inspect video: {path}") from exc

    payload = json.loads(completed.stdout)
    streams = payload.get("streams", [])
    if not streams:
        raise ValueError(f"No video stream found in {path}")
    stream = streams[0]

    frame_count = stream.get("nb_frames")
    if frame_count in (None, "N/A"):
        frame_count = None
    else:
        frame_count = int(frame_count)

    fps_text = stream.get("avg_frame_rate") or stream.get("r_frame_rate")
    fps = float(Fraction(fps_text)) if fps_text and fps_text != "0/0" else 0.0
    return {
        "codec": stream.get("codec_name", "unknown"),
        "width": int(stream["width"]),
        "height": int(stream["height"]),
        "pix_fmt": stream.get("pix_fmt", "unknown"),
        "fps": fps,
        "frame_count": frame_count,
        "duration": float(stream.get("duration") or 0.0),
    }


def read_joint_csv(path: Path) -> tuple[np.ndarray, np.ndarray]:
    if not path.is_file():
        raise FileNotFoundError(f"Missing joint CSV: {path}")

    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"CSV has no header: {path}")
        missing = [column for column in ["timestamp", *JOINT_COLUMNS] if column not in reader.fieldnames]
        if missing:
            raise ValueError(f"CSV is missing columns: {', '.join(missing)}")

        timestamps: list[float] = []
        joints: list[list[float]] = []
        for line_number, row in enumerate(reader, start=2):
            try:
                timestamps.append(float(row["timestamp"]))
                joints.append([float(row[column]) for column in JOINT_COLUMNS])
            except (TypeError, ValueError) as exc:
                raise ValueError(f"Invalid numeric value at CSV line {line_number}") from exc

    if not joints:
        raise ValueError(f"CSV contains no data rows: {path}")
    return np.asarray(timestamps, dtype=np.float64), np.asarray(joints, dtype=np.float64)


def find_cameras(input_dir: Path) -> list[tuple[int, Path]]:
    cameras: list[tuple[int, Path]] = []
    for path in sorted(input_dir.glob("camera_*.mp4")):
        suffix = path.stem.removeprefix("camera_")
        try:
            camera_id = int(suffix)
        except ValueError:
            continue
        cameras.append((camera_id, path))
    if not cameras:
        raise FileNotFoundError(f"No camera_*.mp4 files found in {input_dir}")
    return cameras


def stats_for_array(values: np.ndarray) -> dict[str, Any]:
    values = np.asarray(values, dtype=np.float64)
    if values.ndim == 1:
        values = values[:, None]
    if not np.isfinite(values).all():
        raise ValueError("Cannot calculate stats for NaN or infinite values")
    return {
        "min": values.min(axis=0).astype(float).tolist(),
        "max": values.max(axis=0).astype(float).tolist(),
        "mean": values.mean(axis=0).astype(float).tolist(),
        "std": values.std(axis=0).astype(float).tolist(),
        "count": [int(values.shape[0])],
    }


def video_stats_for_path(path: Path, expected_frames: int) -> dict[str, Any]:
    """Compute LeRobot-style per-channel RGB statistics from sampled video frames."""
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError(
            "Video statistics require OpenCV; install piper_teleop/requirements-dataset.txt"
        ) from exc

    metadata = probe_video(path)
    if metadata["frame_count"] != expected_frames:
        raise ValueError(
            f"Cannot compute video stats for {path}: expected {expected_frames} frames, "
            f"found {metadata["frame_count"]}"
        )
    sample_count = max(100, min(int(expected_frames**0.75), 10_000))
    sample_indices = set(
        np.round(np.linspace(0, expected_frames - 1, sample_count)).astype(int).tolist()
    )

    capture = cv2.VideoCapture(str(path))
    if not capture.isOpened():
        raise RuntimeError(f"Unable to open video for statistics: {path}")

    channel_sum = np.zeros(3, dtype=np.float64)
    channel_sum_sq = np.zeros(3, dtype=np.float64)
    channel_min = np.full(3, np.inf, dtype=np.float64)
    channel_max = np.full(3, -np.inf, dtype=np.float64)
    pixel_count = 0
    try:
        for frame_index in range(expected_frames):
            ok, bgr = capture.read()
            if not ok:
                raise RuntimeError(f"Unable to decode frame {frame_index} from {path}")
            if frame_index not in sample_indices:
                continue
            rgb = bgr[:, :, ::-1].astype(np.float64) / 255.0
            pixels = rgb.reshape(-1, 3)
            channel_sum += pixels.sum(axis=0)
            channel_sum_sq += np.square(pixels).sum(axis=0)
            channel_min = np.minimum(channel_min, pixels.min(axis=0))
            channel_max = np.maximum(channel_max, pixels.max(axis=0))
            pixel_count += pixels.shape[0]
    finally:
        capture.release()

    if pixel_count == 0:
        raise RuntimeError(f"No frames sampled from video: {path}")
    mean = channel_sum / pixel_count
    variance = np.maximum(channel_sum_sq / pixel_count - np.square(mean), 0.0)

    def reshape_channels(values: np.ndarray) -> list[list[list[float]]]:
        return values.astype(float).reshape(3, 1, 1).tolist()

    return {
        "min": reshape_channels(channel_min),
        "max": reshape_channels(channel_max),
        "mean": reshape_channels(mean),
        "std": reshape_channels(np.sqrt(variance)),
        "count": [len(sample_indices)],
    }


def json_dump(path: Path, value: Any) -> None:
    with path.open("w", encoding="utf-8") as handle:
        json.dump(value, handle, indent=2, ensure_ascii=True)
        handle.write("\n")


def jsonl_dump(path: Path, rows: list[dict[str, Any]]) -> None:
    with path.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, ensure_ascii=True, separators=(",", ":")) + "\n")


def transcode_video(source: Path, destination: Path, fps: float) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    run_command(
        [
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            "error",
            "-y",
            "-i",
            str(source),
            "-map",
            "0:v:0",
            "-an",
            "-c:v",
            "libx264",
            "-preset",
            "medium",
            "-crf",
            "18",
            "-pix_fmt",
            "yuv420p",
            "-r",
            f"{fps:g}",
            "-movflags",
            "+faststart",
            str(destination),
        ]
    )


def copy_video(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)


def validate_frame_alignment(
    csv_rows: int,
    camera_metadata: dict[int, dict[str, Any]],
) -> None:
    for camera_id, metadata in camera_metadata.items():
        count = metadata["frame_count"]
        if count is not None and count != csv_rows:
            raise ValueError(
                f"camera_{camera_id}.mp4 has {count} frames but CSV has {csv_rows} rows"
            )


def convert(args: argparse.Namespace) -> dict[str, Any]:
    input_dir = args.input.expanduser().resolve()
    output_dir = args.output.expanduser().resolve()
    if not input_dir.is_dir():
        raise NotADirectoryError(f"Input directory does not exist: {input_dir}")
    if output_dir.exists():
        if not args.overwrite:
            raise FileExistsError(
                f"Output directory already exists: {output_dir}; use --overwrite to replace it"
            )
        if output_dir == input_dir or output_dir in input_dir.parents:
            raise ValueError("Refusing to overwrite the input directory or one of its parents")
        shutil.rmtree(output_dir)

    source_timestamps, source_joints = read_joint_csv(input_dir / "joint_angles.csv")
    cameras = find_cameras(input_dir)
    camera_metadata = {camera_id: probe_video(path) for camera_id, path in cameras}
    validate_frame_alignment(len(source_joints), camera_metadata)

    source_fps = [metadata["fps"] for metadata in camera_metadata.values()]
    if any(fps <= 0 for fps in source_fps):
        raise ValueError("Could not determine a valid FPS for every source video")
    if max(source_fps) - min(source_fps) > 0.01:
        raise ValueError(f"Camera FPS values do not agree: {source_fps}")
    fps_value = float(args.fps if args.fps is not None else round(source_fps[0]))
    fps = int(round(fps_value))
    if abs(fps - fps_value) > 1e-6:
        raise ValueError(f"LeRobot v2.1 requires an integer FPS, got {fps_value}")
    if fps <= 0:
        raise ValueError(f"FPS must be positive, got {fps}")

    if args.angle_unit == "radians":
        joints = np.deg2rad(source_joints).astype(np.float32)
    else:
        joints = source_joints.astype(np.float32)

    frame_count = len(joints)
    frame_indices = np.arange(frame_count, dtype=np.int64)
    video_timestamps = frame_indices.astype(np.float32) / np.float32(fps)
    if args.timestamp_mode == "frame":
        timestamps = video_timestamps
    else:
        timestamps = (source_timestamps - source_timestamps[0]).astype(np.float32)

    chunk_dir = output_dir / "data" / "chunk-000"
    meta_dir = output_dir / "meta"
    chunk_dir.mkdir(parents=True, exist_ok=True)
    meta_dir.mkdir(parents=True, exist_ok=True)

    video_paths: dict[str, str] = {}
    video_codecs: dict[str, str] = {}
    for camera_id, source_path in cameras:
        video_key = f"observation.images.camera_{camera_id}"
        relative_path = (
            Path("videos")
            / "chunk-000"
            / video_key
            / "episode_000000.mp4"
        )
        destination = output_dir / relative_path
        if args.video_mode == "transcode":
            transcode_video(source_path, destination, fps)
            output_metadata = probe_video(destination)
            video_codecs[video_key] = output_metadata["codec"]
        else:
            copy_video(source_path, destination)
            output_metadata = camera_metadata[camera_id]
            video_codecs[video_key] = output_metadata["codec"]
        if output_metadata["frame_count"] is not None and output_metadata["frame_count"] != frame_count:
            raise ValueError(
                f"Converted {source_path.name} has {output_metadata['frame_count']} frames, "
                f"expected {frame_count}"
            )
        video_paths[video_key] = relative_path.as_posix()

    feature_columns: dict[str, pa.Array] = {
        "observation.state": pa.array(joints.tolist(), type=pa.list_(pa.float32())),
        "action": pa.array(joints.tolist(), type=pa.list_(pa.float32())),
        "observation.source_timestamp": pa.array(
            source_timestamps.tolist(), type=pa.float64()
        ),
        "timestamp": pa.array(timestamps.tolist(), type=pa.float32()),
        "episode_index": pa.array([0] * frame_count, type=pa.int64()),
        "frame_index": pa.array(frame_indices.tolist(), type=pa.int64()),
        "index": pa.array(frame_indices.tolist(), type=pa.int64()),
        "task_index": pa.array([0] * frame_count, type=pa.int64()),
    }

    table = pa.table(feature_columns)
    parquet_path = chunk_dir / "episode_000000.parquet"
    pq.write_table(table, parquet_path, compression="zstd")

    joint_names = [
        f"{name}_{args.angle_unit[:-1] if args.angle_unit == 'degrees' else 'rad'}"
        for name in JOINT_COLUMNS
    ]
    features: dict[str, dict[str, Any]] = {
        "observation.state": {
            "dtype": "float32",
            "shape": [len(joint_names)],
            "names": joint_names,
            "info": {"unit": args.angle_unit},
        },
        "action": {
            "dtype": "float32",
            "shape": [len(joint_names)],
            "names": joint_names,
            "info": {
                "unit": args.angle_unit,
                "semantics": "recorded joint command; same source as observation.state",
            },
        },
        "observation.source_timestamp": {
            "dtype": "float64",
            "shape": [1],
            "names": ["source_timestamp"],
            "info": {"unit": "seconds", "source": "joint_angles.csv"},
        },
        "timestamp": {
            "dtype": "float32",
            "shape": [1],
            "names": None,
        },
        "frame_index": {
            "dtype": "int64",
            "shape": [1],
            "names": None,
        },
        "episode_index": {
            "dtype": "int64",
            "shape": [1],
            "names": None,
        },
        "index": {
            "dtype": "int64",
            "shape": [1],
            "names": None,
        },
        "task_index": {
            "dtype": "int64",
            "shape": [1],
            "names": None,
        },
    }
    for camera_id, _ in cameras:
        video_key = f"observation.images.camera_{camera_id}"
        metadata = camera_metadata[camera_id]
        features[video_key] = {
            "dtype": "video",
            "shape": [metadata["height"], metadata["width"], 3],
            "names": ["height", "width", "channel"],
            "info": {
                "video.fps": fps,
                "video.codec": video_codecs[video_key],
                "video.pix_fmt": "yuv420p",
                "video.width": metadata["width"],
                "video.height": metadata["height"],
                "video.channels": 3,
                "video.is_depth_map": False,
                "has_audio": False,
            },
        }

    info = {
        "codebase_version": "v2.1",
        "robot_type": args.robot_type,
        "fps": fps,
        "total_episodes": 1,
        "total_frames": frame_count,
        "total_tasks": 1,
        "total_videos": len(cameras),
        "total_chunks": 1,
        "chunks_size": 1000,
        "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
        "video_path": "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4",
        "splits": {"train": "0:1"},
        "features": features,
        "source": {
            "recording_directory": str(input_dir),
            "joint_csv": "joint_angles.csv",
            "camera_files": [path.name for _, path in cameras],
            "angle_unit": args.angle_unit,
            "timestamp_mode": args.timestamp_mode,
            "video_mode": args.video_mode,
            "action_definition": (
                "action and observation.state both contain the recorded joint command; "
                "no separate measured joint-state stream was available"
            ),
        },
    }
    json_dump(meta_dir / "info.json", info)
    jsonl_dump(
        meta_dir / "episodes.jsonl",
        [
            {
                "episode_index": 0,
                "tasks": [args.task],
                "length": frame_count,
            }
        ],
    )
    jsonl_dump(meta_dir / "tasks.jsonl", [{"task_index": 0, "task": args.task}])
    episode_stats = {
        "observation.state": stats_for_array(joints),
        "action": stats_for_array(joints),
        "observation.source_timestamp": stats_for_array(source_timestamps),
        "timestamp": stats_for_array(timestamps),
        "frame_index": stats_for_array(frame_indices),
        "episode_index": stats_for_array(np.zeros(frame_count)),
        "index": stats_for_array(frame_indices),
        "task_index": stats_for_array(np.zeros(frame_count)),
    }
    for video_key, relative_path in video_paths.items():
        episode_stats[video_key] = video_stats_for_path(
            output_dir / relative_path, frame_count
        )
    jsonl_dump(
        meta_dir / "episodes_stats.jsonl",
        [{"episode_index": 0, "stats": episode_stats}],
    )

    timestamp_deltas = np.diff(source_timestamps)
    gap_indices = np.flatnonzero(timestamp_deltas > (1.5 / fps)).astype(int)
    report = {
        "input": str(input_dir),
        "output": str(output_dir),
        "frames": frame_count,
        "fps": fps,
        "duration_seconds": frame_count / fps,
        "source_timestamp_start": float(source_timestamps[0]),
        "source_timestamp_end": float(source_timestamps[-1]),
        "source_timestamp_gaps": [
            {
                "after_frame_index": int(index),
                "delta_seconds": float(timestamp_deltas[index]),
            }
            for index in gap_indices
        ],
        "camera_metadata": camera_metadata,
        "output_files": {
            "parquet": str(parquet_path.relative_to(output_dir)),
            "videos": list(video_paths.values()),
        },
    }
    json_dump(output_dir / "conversion_report.json", report)
    (output_dir / "README.md").write_text(
        f"""# LeRobot v2.1 Dataset

- Robot: `{args.robot_type}`
- Task: `{args.task}`
- Episodes: 1
- Frames: {frame_count}
- FPS: {fps:g}
- Joint angle unit: `{args.angle_unit}`
- Cameras: {", ".join(video_paths)}

`observation.state` and `action` both contain the recorded 12-joint command
because the source recording has no separate measured joint-state stream.
Video and CSV rows are synchronized by frame index. The original CSV
timestamps are available in `observation.source_timestamp`; the standard
`timestamp` column uses `{args.timestamp_mode}` mode.
""",
        encoding="utf-8",
    )
    return report


def main() -> int:
    args = parse_args()
    try:
        report = convert(args)
    except (FileNotFoundError, FileExistsError, NotADirectoryError, RuntimeError, ValueError) as exc:
        print(f"ERROR: {exc}")
        return 2
    print(json.dumps(report, indent=2, ensure_ascii=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
