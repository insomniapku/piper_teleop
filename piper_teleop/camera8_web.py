#!/usr/bin/env python3
"""Tiny web monitor for a local OpenCV camera."""

from __future__ import annotations

import argparse
import errno
import hmac
import json
import os
import signal
import socket
import sys
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional
from urllib.parse import parse_qs, quote, urlparse

import cv2


class ReusableThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True


def camera_device_path(camera_source: int | str) -> str:
    if isinstance(camera_source, int):
        return f"/dev/video{camera_source}"
    if camera_source.startswith("/dev/video"):
        return camera_source
    return ""


def camera_open_error(camera_label: str, device_path: str) -> str:
    if device_path and not os.path.exists(device_path):
        return f"Camera {camera_label} failed to open: {device_path} does not exist"
    if device_path and not os.access(device_path, os.R_OK | os.W_OK):
        return (
            f"Camera {camera_label} failed to open: no read/write permission for "
            f"{device_path}"
        )
    if device_path:
        return (
            f"Camera {camera_label} failed to open: {device_path} may be busy "
            "or unsupported"
        )
    return f"Camera {camera_label} failed to open"


def local_ip_addresses() -> list[str]:
    addresses: set[str] = set()
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            address = info[4][0]
            if not address.startswith("127."):
                addresses.add(address)
    except socket.gaierror:
        pass

    for target in ("8.8.8.8", "1.1.1.1"):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.connect((target, 80))
                address = sock.getsockname()[0]
                if not address.startswith("127."):
                    addresses.add(address)
        except OSError:
            pass

    return sorted(addresses)


HTML_PAGE = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Camera 8 Live</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #101418;
      --panel: #171d22;
      --ink: #eef3f5;
      --muted: #9eabb3;
      --accent: #32d39b;
      --warn: #f6b44b;
      --line: #28313a;
    }
    * {
      box-sizing: border-box;
    }
    body {
      margin: 0;
      min-height: 100vh;
      background: var(--bg);
      color: var(--ink);
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }
    main {
      min-height: 100vh;
      display: grid;
      grid-template-rows: auto 1fr;
    }
    header {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 16px;
      padding: 14px 18px;
      background: var(--panel);
      border-bottom: 1px solid var(--line);
    }
    h1 {
      margin: 0;
      font-size: 18px;
      font-weight: 650;
      letter-spacing: 0;
    }
    .status {
      display: flex;
      align-items: center;
      gap: 8px;
      color: var(--muted);
      font-size: 13px;
      white-space: nowrap;
    }
    .dot {
      width: 9px;
      height: 9px;
      border-radius: 50%;
      background: var(--warn);
      box-shadow: 0 0 0 3px rgba(246, 180, 75, 0.14);
    }
    .status.ready .dot {
      background: var(--accent);
      box-shadow: 0 0 0 3px rgba(50, 211, 155, 0.14);
    }
    .stage {
      min-height: 0;
      display: grid;
      place-items: center;
      padding: 18px;
    }
    .frame {
      width: min(100%, 1280px);
      aspect-ratio: 4 / 3;
      background: #050708;
      border: 1px solid var(--line);
      overflow: hidden;
    }
    img {
      width: 100%;
      height: 100%;
      display: block;
      object-fit: contain;
    }
    @media (max-width: 720px) {
      header {
        align-items: flex-start;
        flex-direction: column;
      }
      .stage {
        padding: 10px;
      }
    }
  </style>
</head>
<body>
  <main>
    <header>
      <h1>Camera 8 Live</h1>
      <div class="status" id="status"><span class="dot"></span><span id="statusText">Starting</span></div>
    </header>
    <section class="stage">
      <div class="frame">
        <img id="stream" alt="Camera 8 live stream">
      </div>
    </section>
  </main>
  <script>
    const statusEl = document.getElementById("status");
    const statusText = document.getElementById("statusText");
    const streamEl = document.getElementById("stream");
    const token = new URLSearchParams(window.location.search).get("token") || "";
    const authQuery = token ? `?token=${encodeURIComponent(token)}` : "";
    streamEl.src = `/stream.mjpg${authQuery}`;

    async function refreshStatus() {
      try {
        const response = await fetch(`/status.json${authQuery}`, { cache: "no-store" });
        const data = await response.json();
        statusEl.classList.toggle("ready", Boolean(data.ready));
        statusText.textContent = data.ready
          ? `${data.width}x${data.height} @ ${data.fps.toFixed(1)} fps`
          : data.message;
      } catch (error) {
        statusEl.classList.remove("ready");
        statusText.textContent = "Offline";
      }
    }

    refreshStatus();
    setInterval(refreshStatus, 1000);
  </script>
</body>
</html>
"""


class CameraStream:
    def __init__(
        self,
        camera_source: int | str,
        width: int,
        height: int,
        fps: int,
        jpeg_quality: int,
        retry_interval: float,
    ) -> None:
        self.camera_source = camera_source
        self.camera_label = str(camera_source)
        self.device_path = camera_device_path(camera_source)
        self.width = width
        self.height = height
        self.target_fps = fps
        self.jpeg_quality = jpeg_quality
        self.retry_interval = retry_interval
        self.lock = threading.Lock()
        self.frame_ready = threading.Condition(self.lock)
        self.latest_jpeg: Optional[bytes] = None
        self.last_error = "Starting"
        self.open_attempts = 0
        self._last_logged_error = ""
        self._last_logged_error_time = 0.0
        self.capture_width = 0
        self.capture_height = 0
        self.frame_count = 0
        self.last_frame_time = 0.0
        self._fps_window_start = time.monotonic()
        self._fps_window_frames = 0
        self.measured_fps = 0.0
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        self.thread.join(timeout=2.0)

    def status(self) -> dict:
        with self.lock:
            ready = self.latest_jpeg is not None and time.monotonic() - self.last_frame_time < 2.0
            return {
                "camera": self.camera_label,
                "device_path": self.device_path,
                "ready": ready,
                "message": "Live" if ready else self.last_error,
                "width": self.capture_width or self.width,
                "height": self.capture_height or self.height,
                "fps": self.measured_fps,
                "frames": self.frame_count,
                "open_attempts": self.open_attempts,
            }

    def wait_for_jpeg(self, last_seen: Optional[bytes], timeout: float = 1.0) -> Optional[bytes]:
        deadline = time.monotonic() + timeout
        with self.frame_ready:
            while self.latest_jpeg is None or self.latest_jpeg is last_seen:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0 or self.stop_event.is_set():
                    return self.latest_jpeg
                self.frame_ready.wait(timeout=remaining)
            return self.latest_jpeg

    def _run(self) -> None:
        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        min_period = 1.0 / max(float(self.target_fps), 1.0)

        while not self.stop_event.is_set():
            cap: Optional[cv2.VideoCapture] = None
            with self.lock:
                self.open_attempts += 1
                self.last_error = f"Opening camera {self.camera_label}"
            try:
                cap = cv2.VideoCapture(self.camera_source, cv2.CAP_V4L2)
            except Exception as exc:
                self._mark_camera_error(f"Camera open raised {type(exc).__name__}: {exc}")
                self._sleep_before_retry()
                continue

            if not cap.isOpened():
                if cap is not None:
                    cap.release()
                self._mark_camera_error(camera_open_error(self.camera_label, self.device_path))
                self._sleep_before_retry()
                continue

            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_FPS, self.target_fps)
            print(f"[camera] opened {self.camera_label}")

            while not self.stop_event.is_set():
                started = time.monotonic()
                ok, frame = cap.read()
                if not ok or frame is None:
                    self._mark_camera_error(f"Camera {self.camera_label} read failed; retrying")
                    time.sleep(0.05)
                    break

                ok, encoded = cv2.imencode(".jpg", frame, encode_params)
                if not ok:
                    self._mark_camera_error("JPEG encode failed")
                    continue

                now = time.monotonic()
                with self.frame_ready:
                    self.latest_jpeg = encoded.tobytes()
                    self.capture_height, self.capture_width = frame.shape[:2]
                    self.last_error = "Live"
                    self.last_frame_time = now
                    self.frame_count += 1
                    self._fps_window_frames += 1
                    elapsed = now - self._fps_window_start
                    if elapsed >= 1.0:
                        self.measured_fps = self._fps_window_frames / elapsed
                        self._fps_window_start = now
                        self._fps_window_frames = 0
                    self.frame_ready.notify_all()

                remaining = min_period - (time.monotonic() - started)
                if remaining > 0.0:
                    time.sleep(remaining)
            if cap is not None:
                cap.release()
            self._sleep_before_retry()

    def _mark_camera_error(self, message: str) -> None:
        with self.frame_ready:
            self.last_error = message
            self.latest_jpeg = None
            self.frame_ready.notify_all()
        now = time.monotonic()
        if message != self._last_logged_error or now - self._last_logged_error_time >= 5.0:
            print(f"[camera] {message}", file=sys.stderr)
            self._last_logged_error = message
            self._last_logged_error_time = now

    def _sleep_before_retry(self) -> None:
        self.stop_event.wait(max(0.1, self.retry_interval))


def make_handler(
    stream: CameraStream,
    auth_token: str = "",
) -> type[BaseHTTPRequestHandler]:
    class CameraRequestHandler(BaseHTTPRequestHandler):
        server_version = "Camera8Web/1.0"

        def log_message(self, fmt: str, *args: object) -> None:
            sys.stderr.write(
                "%s - - [%s] %s\n"
                % (self.address_string(), self.log_date_time_string(), fmt % args)
            )

        def do_GET(self) -> None:
            parsed = urlparse(self.path)
            if parsed.path == "/favicon.ico":
                self.send_response(HTTPStatus.NO_CONTENT)
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                return

            if not self._authorized(parsed.query):
                self._send_bytes(
                    b"Unauthorized: open the full URL printed by the server, including ?token=...\n",
                    "text/plain; charset=utf-8",
                    status=HTTPStatus.UNAUTHORIZED,
                )
                return

            if parsed.path in ("/", "/index.html"):
                self._send_bytes(HTML_PAGE.encode("utf-8"), "text/html; charset=utf-8")
                return
            if parsed.path == "/status.json":
                payload = json.dumps(stream.status()).encode("utf-8")
                self._send_bytes(payload, "application/json; charset=utf-8")
                return
            if parsed.path == "/snapshot.jpg":
                jpeg = stream.wait_for_jpeg(None, timeout=2.0)
                if jpeg is None:
                    self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "No frame available")
                    return
                self._send_bytes(jpeg, "image/jpeg")
                return
            if parsed.path == "/stream.mjpg":
                self._send_stream()
                return
            self.send_error(HTTPStatus.NOT_FOUND)

        def _authorized(self, query: str) -> bool:
            if not auth_token:
                return True
            tokens = parse_qs(query).get("token", [""])
            return hmac.compare_digest(tokens[-1], auth_token)

        def _send_bytes(
            self,
            payload: bytes,
            content_type: str,
            status: HTTPStatus = HTTPStatus.OK,
        ) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(payload)

        def _send_stream(self) -> None:
            boundary = "camera-frame"
            self.send_response(HTTPStatus.OK)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={boundary}")
            self.end_headers()

            last_seen: Optional[bytes] = None
            while not stream.stop_event.is_set():
                jpeg = stream.wait_for_jpeg(last_seen, timeout=2.0)
                if jpeg is None:
                    continue
                last_seen = jpeg
                try:
                    self.wfile.write(f"--{boundary}\r\n".encode("ascii"))
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii"))
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    break

    return CameraRequestHandler


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Serve a camera as a browser MJPEG stream.")
    parser.add_argument("--camera-index", type=int, default=8)
    parser.add_argument(
        "--camera-device",
        default="",
        help="Open a device path directly, for example /dev/video8",
    )
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8222)
    parser.add_argument(
        "--strict-port",
        action="store_true",
        help="Fail instead of trying the next port when --port is occupied",
    )
    parser.add_argument(
        "--port-search-limit",
        type=int,
        default=50,
        help="How many consecutive ports to try when the preferred port is occupied",
    )
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--jpeg-quality", type=int, default=82)
    parser.add_argument(
        "--retry-interval",
        type=float,
        default=1.0,
        help="Seconds between camera reopen attempts after open/read failure",
    )
    parser.add_argument(
        "--auth-token",
        default=os.environ.get("CAMERA8_TOKEN", ""),
        help="Require ?token=... for every page and stream request",
    )
    args = parser.parse_args()
    if not 0 <= args.port <= 65535:
        parser.error("--port must be in [0, 65535]")
    if args.port_search_limit <= 0:
        parser.error("--port-search-limit must be positive")
    if args.retry_interval <= 0.0:
        parser.error("--retry-interval must be positive")
    return args


def create_server(
    host: str,
    preferred_port: int,
    handler_class: type[BaseHTTPRequestHandler],
    *,
    strict_port: bool,
    port_search_limit: int,
) -> ThreadingHTTPServer:
    if preferred_port == 0:
        return ReusableThreadingHTTPServer((host, 0), handler_class)

    attempts = 1 if strict_port else max(1, port_search_limit)
    last_error: Optional[OSError] = None
    for offset in range(attempts):
        port = preferred_port + offset
        if port > 65535:
            break
        try:
            return ReusableThreadingHTTPServer((host, port), handler_class)
        except OSError as exc:
            last_error = exc
            if exc.errno != errno.EADDRINUSE:
                raise

    assert last_error is not None
    end_port = min(65535, preferred_port + attempts - 1)
    raise RuntimeError(
        f"No free port found from {preferred_port} to "
        f"{end_port}"
    ) from last_error


def main() -> int:
    args = parse_args()
    camera_source: int | str = args.camera_device or args.camera_index
    stream = CameraStream(
        camera_source=camera_source,
        width=args.width,
        height=args.height,
        fps=args.fps,
        jpeg_quality=args.jpeg_quality,
        retry_interval=args.retry_interval,
    )
    server = create_server(
        args.host,
        args.port,
        make_handler(stream, args.auth_token),
        strict_port=args.strict_port,
        port_search_limit=args.port_search_limit,
    )
    server.daemon_threads = True
    stream.start()

    def request_exit(_signum: int, _frame: object) -> None:
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, request_exit)
    signal.signal(signal.SIGTERM, request_exit)

    bound_port = int(server.server_address[1])
    if args.port != 0 and bound_port != args.port:
        print(f"Port {args.port} is busy; using {bound_port} instead.")
    token_query = f"?token={quote(args.auth_token, safe='')}" if args.auth_token else ""
    print(f"Serving camera {camera_source} at http://127.0.0.1:{bound_port}/{token_query}")
    if args.host in ("0.0.0.0", ""):
        for address in local_ip_addresses():
            print(f"LAN URL: http://{address}:{bound_port}/{token_query}")
    elif args.host.startswith("127."):
        print("Bound to localhost only; other devices cannot access this server.")
    if not args.auth_token:
        print("[security] No --auth-token set; anyone with the URL can view this stream.")
    time.sleep(1.0)
    initial_status = stream.status()
    if not initial_status["ready"]:
        print(f"[camera] {initial_status['message']}", file=sys.stderr)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("Stopping camera web server.")
    finally:
        stream.stop()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
