#!/usr/bin/env python3
"""Inspect an existing recording's capture cadence without opening hardware."""

import argparse
import csv
import json
from pathlib import Path


def analyze(recording_dir, fps=30.0):
    directory = Path(recording_dir)
    with (directory / "joint_angles.csv").open(newline="", encoding="utf-8") as source:
        rows = list(csv.DictReader(source))
    report_path = directory / "sync_report.json"
    sync = json.loads(report_path.read_text(encoding="utf-8")) if report_path.exists() else {}
    fps = float(sync.get("fps", fps))
    if fps <= 0:
        raise ValueError("FPS must be positive")
    stamps = [float(row["timestamp"]) for row in rows]
    deltas = [b-a for a, b in zip(stamps, stamps[1:])]
    ordered = sorted(deltas)
    gaps = [{"after_frame": i, "interval_ms": value*1000}
            for i, value in enumerate(deltas) if value > 1.5/fps]
    nonincreasing = [i for i, value in enumerate(deltas) if value <= 0]
    span = stamps[-1]-stamps[0] if len(stamps) > 1 else 0
    return {
        "recording": str(directory), "frames": len(rows), "nominal_fps": fps,
        "effective_capture_fps": (len(rows)-1)/span if span > 0 else None,
        "capture_span_seconds": span,
        "fixed_fps_playback_span_seconds": max(0, len(rows)-1)/fps,
        "interval_p50_ms": ordered[(len(ordered)-1)//2]*1000 if ordered else None,
        "interval_p95_ms": ordered[int((len(ordered)-1)*0.95)]*1000 if ordered else None,
        "interval_max_ms": max(deltas)*1000 if deltas else None,
        "long_gap_count": len(gaps), "first_long_gaps": gaps[:20],
        "nonincreasing_timestamp_count": len(nonincreasing),
        "cadence_ok": bool(len(rows) > 1 and not gaps and not nonincreasing),
        "sync_report": sync,
        "note": "Capture cadence only. This does not certify camera exposure timing, video decoding, or motion smoothness.",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("--fps", type=float, default=30.0)
    args = parser.parse_args()
    report = analyze(args.recording, args.fps)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["cadence_ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
