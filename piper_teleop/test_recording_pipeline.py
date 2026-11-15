"""Offline regressions: fake cameras/feedback only; no XR, CAN, or motion."""

from collections import deque
import csv
import io
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import types
import unittest

import numpy as np

# Minimal video-interface types allow timing tests without installing OpenCV.
# Actual video decoding/encoding is deliberately outside these fake-camera tests.
try:
    import cv2
except ImportError:
    cv2 = types.ModuleType("cv2")
    cv2.VideoCapture = type("VideoCapture", (), {})
    cv2.VideoWriter = type("VideoWriter", (), {})
    sys.modules["cv2"] = cv2

from teleop_recording import RecordingService
from trajectory_recorder import CameraFrame, RecordingSession, TrajectoryRecorder
from scripts.analyze_recording_timing import analyze


class FakeRecorder:
    fps = 30

    def __init__(self, slow_start=False):
        self.is_recording = False
        self.states = []
        self.frames = []
        self.started = threading.Event()
        self.start_entered = threading.Event()
        self.start_gate = threading.Event()
        if not slow_start:
            self.start_gate.set()
        self.stopped = threading.Event()
        self.cleaned = False

    def start_recording(self):
        self.start_entered.set()
        self.start_gate.wait(2.0)
        self.is_recording = True
        self.started.set()
        return True

    def stop_recording(self):
        self.is_recording = False
        self.stopped.set()

    def record_state(self, *args):
        self.states.append(args)

    def record_frame(self, timestamp_ns):
        self.frames.append(timestamp_ns)

    def cleanup(self):
        if self.is_recording:
            self.stop_recording()
        self.cleaned = True


def feedback():
    stamp = time.monotonic_ns()
    return np.zeros(6), stamp, stamp / 1e9, stamp


class RecordingServiceTests(unittest.TestCase):
    def test_sampling_continues_when_main_control_is_blocked(self):
        recorder = FakeRecorder()
        service = RecordingService(recorder, feedback, feedback)
        service.start()
        try:
            service.request_toggle()
            self.assertTrue(recorder.started.wait(1.0))
            # Simulate one long IK/I/O call in the main thread.
            threading.Event().wait(0.35)
            service.request_stop()
            self.assertTrue(recorder.stopped.wait(1.0))
            self.assertGreaterEqual(len(recorder.frames), 6)
            self.assertGreaterEqual(len(recorder.states), 12)
            self.assertTrue(all(b > a for a, b in zip(recorder.frames, recorder.frames[1:])))
            self.assertLess(max(np.diff(recorder.frames)), 150_000_000)
        finally:
            service.close()
        self.assertTrue(recorder.cleaned)

    def test_slow_camera_start_is_not_in_button_handler(self):
        recorder = FakeRecorder(slow_start=True)
        service = RecordingService(recorder, feedback, feedback)
        service.start()
        try:
            service.request_toggle()
            self.assertTrue(recorder.start_entered.wait(1.0))
            # This call returns even while start_recording is held behind a gate.
            service.request_stop()
            self.assertFalse(service._sampling.is_set())
            recorder.start_gate.set()
            self.assertTrue(recorder.stopped.wait(1.0))
            self.assertEqual(recorder.states, [])
        finally:
            recorder.start_gate.set()
            service.close()

    def test_missing_feedback_never_becomes_command_state(self):
        recorder = FakeRecorder()
        service = RecordingService(recorder, lambda: None, feedback)
        service.start()
        try:
            service.request_toggle()
            self.assertTrue(recorder.started.wait(1.0))
            threading.Event().wait(0.12)
        finally:
            service.close()
        self.assertEqual(recorder.states, [])
        self.assertGreater(service.snapshot_stats().get("missing_feedback", 0), 0)

    def test_stale_or_skewed_feedback_is_rejected(self):
        for offset, expected in [(1_000_000_000, "stale_feedback"),
                                 (50_000_000, "arm_timestamp_skew")]:
            recorder = FakeRecorder()
            def older():
                sample = feedback()
                return sample[0], sample[1]-offset, sample[2], sample[3]
            service = RecordingService(recorder, older, feedback)
            service.start()
            try:
                service.request_toggle()
                self.assertTrue(recorder.started.wait(1.0))
                threading.Event().wait(0.08)
            finally:
                service.close()
            self.assertEqual(recorder.states, [])
            self.assertGreater(service.snapshot_stats().get(expected, 0), 0)

    def test_sampling_stops_before_async_flush(self):
        recorder = FakeRecorder()
        service = RecordingService(recorder, feedback, feedback)
        service.start()
        try:
            service.request_toggle()
            self.assertTrue(recorder.started.wait(1.0))
            threading.Event().wait(0.05)
            service.request_stop()
            self.assertFalse(service._sampling.is_set())
            self.assertTrue(recorder.stopped.wait(1.0))
            n = len(recorder.states)
            threading.Event().wait(0.05)
            self.assertEqual(len(recorder.states), n)
        finally:
            service.close()

    def test_duplicate_feedback_is_counted_without_making_new_state_samples(self):
        recorder = FakeRecorder()
        fixed = feedback()
        service = RecordingService(recorder, lambda: fixed, lambda: fixed)
        service.start()
        try:
            service.request_toggle()
            self.assertTrue(recorder.started.wait(1.0))
            threading.Event().wait(0.08)
        finally:
            service.close()
        self.assertEqual(len(recorder.states), 1)
        self.assertGreater(service.snapshot_stats().get("duplicate_feedback", 0), 0)


class RecorderTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.recorder = TrajectoryRecorder(recording_dir=self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def test_full_queue_keeps_latest_requests_without_blocking(self):
        recorder = self.recorder
        recorder.session = RecordingSession(Path(self.temp.name), 0, Path("unused"))
        recorder.is_recording = True
        for i in range(10):
            recorder.record_frame(i)
        self.assertEqual(recorder.record_queue.qsize(), 2)
        kept = []
        while not recorder.record_queue.empty():
            kept.append(recorder.record_queue.get_nowait().timestamp_ns)
            recorder.record_queue.task_done()
        self.assertEqual(kept, [8, 9])
        self.assertEqual(recorder.session.queue_full_count, 8)
        self.assertEqual(recorder.record_queue.unfinished_tasks, 0)

    def test_sync_tries_next_reference_instead_of_waiting_on_bad_one(self):
        recorder = self.recorder
        target = time.monotonic_ns()
        image = np.zeros((2, 2, 3), np.uint8)
        recorder.camera_buffers = {
            0: deque([CameraFrame(target, 1, image),
                      CameraFrame(target+33_000_000, 2, image)]),
            2: deque([CameraFrame(target+33_000_000, 1, image)]),
        }
        stamp = target + 33_000_000
        recorder.record_state(np.zeros(6), np.zeros(6), stamp, stamp)
        result = recorder._wait_for_synchronized_bundle(target, [0, 2])
        self.assertIsNotNone(result)
        self.assertEqual(result[1][0].sequence, 2)
        self.assertEqual(result[2], stamp)

    def test_incompatible_frames_are_not_duplicated_or_interpolated(self):
        recorder = self.recorder
        recorder.camera_wait_timeout_ns = 1_000_000
        target = time.monotonic_ns()
        image = np.zeros((2, 2, 3), np.uint8)
        recorder.camera_buffers = {
            0: deque([CameraFrame(target, 1, image)]),
            2: deque([CameraFrame(target+60_000_000, 1, image)]),
        }
        recorder.record_state(np.zeros(6), np.zeros(6), target, target)
        self.assertIsNone(recorder._wait_for_synchronized_bundle(target, [0, 2]))
        self.assertEqual(recorder.last_written_sequences, {})

    def test_old_requests_are_dropped_without_encoding(self):
        recorder = self.recorder
        session = RecordingSession(Path(self.temp.name), 0, Path("unused"))
        from trajectory_recorder import RecordingRequest
        recorder.record_queue.put(RecordingRequest(time.monotonic_ns()-1_000_000_000))
        recorder.record_queue.put(None)
        recorder._writer_loop(session)
        self.assertEqual(session.stale_request_count, 1)
        self.assertEqual(session.frame_count, 0)
        self.assertEqual(recorder.record_queue.unfinished_tasks, 0)

    def test_writer_reports_capture_gaps_and_preserves_csv_alignment(self):
        recorder = self.recorder
        csv_file = io.StringIO()
        now = time.monotonic_ns()
        session = RecordingSession(Path(self.temp.name), now, Path("unused"),
                                   csv_file=csv_file, csv_writer=csv.writer(csv_file))
        class Writer:
            def __init__(self): self.images = []
            def write(self, image): self.images.append(image.copy())
        video = Writer()
        session.video_writers = {0: video}
        image = np.zeros((2, 2, 3), np.uint8)
        results = []
        for i, gap in enumerate([0, 100_000_000]):
            stamp = now + gap
            recorder.record_state(np.zeros(6), np.zeros(6), stamp, stamp)
            results.append((recorder.state_buffer[-1], {0: CameraFrame(stamp, i+1, image)}, stamp))
        recorder._wait_for_synchronized_bundle = lambda *_: results.pop(0)
        from trajectory_recorder import RecordingRequest
        recorder.record_queue.put(RecordingRequest(now))
        recorder.record_queue.put(RecordingRequest(now))
        thread = threading.Thread(target=recorder._writer_loop, args=(session,))
        thread.start()
        recorder.record_queue.join()
        recorder.record_queue.put(None)
        thread.join(1.0)
        self.assertFalse(thread.is_alive())
        rows = list(csv.reader(io.StringIO(csv_file.getvalue())))
        self.assertEqual(len(rows), len(video.images))
        self.assertEqual(len(rows), 2)
        self.assertEqual([int(row[-3]) for row in rows], [0, 1])
        self.assertAlmostEqual(float(rows[1][-2]), 0.1)
        self.assertEqual(session.long_frame_gaps, 1)
        self.assertEqual(session.max_frame_interval_ns, 100_000_000)

    def test_diagnostic_exposes_gaps_without_retiming_the_recording(self):
        directory = Path(self.temp.name)
        source = "timestamp\n0.0\n0.033333\n0.133333\n"
        (directory / "joint_angles.csv").write_text(source, encoding="utf-8")
        report = analyze(directory)
        self.assertFalse(report["cadence_ok"])
        self.assertEqual(report["long_gap_count"], 1)
        self.assertAlmostEqual(report["interval_max_ms"], 100.0)
        self.assertEqual((directory / "joint_angles.csv").read_text(encoding="utf-8"), source)

    def test_diagnostic_rejects_duplicate_timestamps(self):
        directory = Path(self.temp.name)
        (directory / "joint_angles.csv").write_text("timestamp\n0\n0\n0.033333\n", encoding="utf-8")
        report = analyze(directory)
        self.assertFalse(report["cadence_ok"])
        self.assertEqual(report["nonincreasing_timestamp_count"], 1)


if __name__ == "__main__":
    unittest.main()
