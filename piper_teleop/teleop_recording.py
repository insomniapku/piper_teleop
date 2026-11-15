"""Feedback-only recording workers; this module never sends robot commands."""

from collections import Counter, deque
import threading
import time


class RecordingService:
    """Keep feedback cadence and slow session I/O out of the IK/control loop.

    Only the lifecycle worker starts/stops the recorder. The sampler owns the
    feedback reads. Main-thread button handlers only change the desired state.
    """

    def __init__(self, recorder, left_feedback, right_feedback, sample_rate=60.0):
        if sample_rate <= 0 or recorder.fps <= 0 or sample_rate < recorder.fps:
            raise ValueError("feedback sample rate must be >= positive camera FPS")
        self.recorder = recorder
        self.feedback = (left_feedback, right_feedback)
        self.sample_rate = float(sample_rate)
        self.max_feedback_age_ns = 200_000_000
        self.max_arm_skew_ns = 20_000_000
        self._condition = threading.Condition()
        self._desired = False
        self._shutdown = threading.Event()
        self._sampling = threading.Event()
        self._stats_lock = threading.Lock()
        self._stats = Counter()
        self._control_ms = deque(maxlen=2048)
        self._sample_gaps_ms = deque(maxlen=2048)
        self._grippers = (0.0, 0.0)
        self._threads = []
        self.recorder.service_stats_provider = self.snapshot_stats

    def start(self):
        if self._threads:
            raise RuntimeError("recording service already started")
        for name, target in (("piper-recording-lifecycle", self._lifecycle),
                             ("piper-feedback-sampler", self._sample_loop)):
            thread = threading.Thread(name=name, target=target, daemon=True)
            self._threads.append(thread)
            thread.start()

    def request_toggle(self):
        with self._condition:
            self._desired = not self._desired
            if not self._desired:
                self._sampling.clear()
            self._condition.notify_all()

    def request_stop(self):
        with self._condition:
            self._desired = False
            self._sampling.clear()
            self._condition.notify_all()

    def update_grippers(self, left, right):
        with self._stats_lock:
            self._grippers = (float(left or 0.0), float(right or 0.0))

    def observe_control_cycle(self, elapsed_seconds):
        with self._stats_lock:
            self._control_ms.append(elapsed_seconds * 1000.0)
            self._stats["control_cycles"] += 1
            if elapsed_seconds > 1.0 / self.sample_rate:
                self._stats["control_overruns"] += 1

    def _count(self, name, amount=1):
        with self._stats_lock:
            self._stats[name] += amount

    @staticmethod
    def _distribution(values):
        values = sorted(values)
        if not values:
            return {"samples": 0, "p50_ms": None, "p95_ms": None, "max_ms": None}
        return {"samples": len(values), "p50_ms": values[(len(values) - 1) // 2],
                "p95_ms": values[int((len(values) - 1) * 0.95)], "max_ms": values[-1]}

    def snapshot_stats(self):
        with self._stats_lock:
            return {"scope": "service lifetime; distributions use last 2048 samples",
                    "feedback_rate_hz": self.sample_rate, **dict(self._stats),
                    "control_work": self._distribution(self._control_ms),
                    "feedback_sample_interval": self._distribution(self._sample_gaps_ms)}

    def _lifecycle(self):
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._shutdown.is_set() or
                                         self._desired != self.recorder.is_recording)
                desired = self._desired and not self._shutdown.is_set()
            try:
                if desired:
                    started = self.recorder.start_recording()
                    with self._condition:
                        if started and self._desired and not self._shutdown.is_set():
                            self._sampling.set()
                        elif not started:
                            self._desired = False
                else:
                    self._sampling.clear()
                    if self.recorder.is_recording:
                        self.recorder.stop_recording()
            except Exception as exc:
                self._count("lifecycle_errors")
                self._sampling.clear()
                with self._condition:
                    self._desired = False
                print(f"[RECORDER] Session I/O failed: {exc}")
                # Do not spin or silently restart a failed recording.
                return
            if self._shutdown.is_set():
                return

    def _sample_loop(self):
        period = 1.0 / self.sample_rate
        frame_period_ns = int(round(1e9 / self.recorder.fps))
        deadline = time.monotonic()
        next_frame_ns = None
        last_pair = None
        last_sample_ns = None
        while not self._shutdown.is_set():
            if self._sampling.is_set():
                now_ns = time.monotonic_ns()
                try:
                    pair = tuple(read() for read in self.feedback)
                    if any(sample is None for sample in pair):
                        self._count("missing_feedback")
                    else:
                        left, right = pair
                        timestamps = (left[1], right[1])
                        if any(now_ns - stamp > self.max_feedback_age_ns or
                               stamp - now_ns > 20_000_000 for stamp in timestamps):
                            self._count("stale_feedback")
                        elif abs(timestamps[0] - timestamps[1]) > self.max_arm_skew_ns:
                            self._count("arm_timestamp_skew")
                        elif timestamps == last_pair:
                            self._count("duplicate_feedback")
                        elif self._sampling.is_set():
                            with self._stats_lock:
                                grippers = self._grippers
                            self.recorder.record_state(
                                left[0], right[0], left[1], right[1], left[2], right[2],
                                left[3], right[3], grippers[0], grippers[1])
                            last_pair = timestamps
                            self._count("feedback_samples")
                            with self._stats_lock:
                                if last_sample_ns is not None:
                                    self._sample_gaps_ms.append((now_ns-last_sample_ns)/1e6)
                            last_sample_ns = now_ns
                    if next_frame_ns is None:
                        next_frame_ns = now_ns
                    if now_ns >= next_frame_ns and self._sampling.is_set():
                        skipped = (now_ns - next_frame_ns) // frame_period_ns
                        self._count("missed_recording_deadlines", int(skipped))
                        self.recorder.record_frame(
                            timestamp_ns=next_frame_ns + skipped * frame_period_ns)
                        next_frame_ns += (skipped + 1) * frame_period_ns
                except Exception as exc:
                    self._count("feedback_read_errors")
                    # A broken feedback reader must not flood the terminal.
                    if self.snapshot_stats().get("feedback_read_errors", 0) == 1:
                        print(f"[RECORDER] Feedback read failed: {exc}")
            else:
                next_frame_ns = last_pair = last_sample_ns = None
            deadline += period
            remaining = deadline - time.monotonic()
            if remaining < 0:
                self._count("sampler_overruns")
                deadline = time.monotonic()
                remaining = period
            self._shutdown.wait(remaining)

    def close(self, timeout=10.0):
        self.request_stop()
        self._shutdown.set()
        with self._condition:
            self._condition.notify_all()
        for thread in self._threads:
            thread.join(timeout=timeout)
        if any(thread.is_alive() for thread in self._threads):
            raise RuntimeError("recording worker is still finishing I/O; resources kept open")
        self.recorder.cleanup()
