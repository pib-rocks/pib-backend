"""HTTP MJPEG and single-frame JPEG server for the OAK-D Lite camera node (PR-2140).

Serves the hardware-encoded JPEG bitstream from the OAK-D Lite directly over
HTTP (`multipart/x-mixed-replace; boundary=frame` on `/stream` and `image/jpeg`
on `/snapshot`) without base64 or rosbridge overhead.
"""

from __future__ import annotations

from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import threading
import time
from typing import Optional, Tuple
from urllib.parse import urlparse

DEFAULT_STREAM_HOST = "0.0.0.0"
DEFAULT_STREAM_PORT = 8081
STREAM_BOUNDARY = "frame"


class MjpegFrameBuffer:
    """Thread-safe holder for the latest hardware-encoded JPEG frame."""

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._jpeg_bytes: bytes = b""
        self._sequence: int = 0
        self._timestamp: float = 0.0
        self._available: bool = True
        self._closed: bool = False

    @property
    def sequence(self) -> int:
        with self._condition:
            return self._sequence

    @property
    def latest_jpeg(self) -> bytes:
        with self._condition:
            return self._jpeg_bytes

    def publish(self, jpeg_bytes: bytes) -> None:
        if not jpeg_bytes:
            return
        with self._condition:
            self._jpeg_bytes = bytes(jpeg_bytes)
            self._sequence += 1
            self._timestamp = time.monotonic()
            self._available = True
            self._condition.notify_all()

    def set_available(self, available: bool) -> None:
        with self._condition:
            self._available = bool(available)
            self._condition.notify_all()

    def close(self) -> None:
        with self._condition:
            self._closed = True
            self._condition.notify_all()

    def wait_for_next(
        self, last_sequence: int, timeout: float = 2.0
    ) -> Tuple[Optional[bytes], int, bool]:
        """Wait for a frame newer than ``last_sequence``.

        Returns ``(jpeg_bytes, sequence, available)``. When the timeout elapses
        without a new frame, returns ``(None, last_sequence, available)``.
        """
        deadline = time.monotonic() + max(0.01, float(timeout))
        with self._condition:
            while (
                not self._closed and self._available and self._sequence == last_sequence
            ):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None, self._sequence, self._available
                self._condition.wait(timeout=remaining)
            if self._closed or not self._available:
                return None, self._sequence, False
            return self._jpeg_bytes, self._sequence, True


class _MjpegRequestHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, format: str, *args) -> None:  # noqa: A002
        # Suppress per-frame/per-request stdout noise inside the ROS container.
        return

    @property
    def frame_buffer(self) -> MjpegFrameBuffer:
        return self.server.frame_buffer  # type: ignore[attr-defined]

    def do_GET(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        path = parsed.path.rstrip("/") or "/"
        if path in ("/stream", "/api/camera/stream"):
            self._handle_stream()
            return
        if path in ("/snapshot", "/api/camera/snapshot"):
            self._handle_snapshot()
            return
        if path in ("/status", "/api/camera/status"):
            self._handle_status()
            return
        self._send_json(HTTPStatus.NOT_FOUND, {"error": "Not found"})

    def _handle_status(self) -> None:
        buf = self.frame_buffer
        jpeg = buf.latest_jpeg
        self._send_json(
            HTTPStatus.OK,
            {
                "available": bool(jpeg),
                "sequence": buf.sequence,
                "frameBytes": len(jpeg),
            },
        )

    def _handle_snapshot(self) -> None:
        jpeg = self.frame_buffer.latest_jpeg
        if not jpeg:
            self._send_json(
                HTTPStatus.SERVICE_UNAVAILABLE,
                {"error": "Camera frame not available"},
            )
            return
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(jpeg)))
        self.send_header(
            "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0"
        )
        self.send_header("Pragma", "no-cache")
        self.send_header("Connection", "close")
        self.end_headers()
        self.wfile.write(jpeg)

    def _handle_stream(self) -> None:
        buf = self.frame_buffer
        # Wait briefly for the first frame if the pipeline just started.
        jpeg = buf.latest_jpeg
        seq = buf.sequence
        if not jpeg:
            jpeg, seq, available = buf.wait_for_next(0, timeout=2.0)
            if not available or not jpeg:
                self._send_json(
                    HTTPStatus.SERVICE_UNAVAILABLE,
                    {"error": "Camera stream not available"},
                )
                return

        self.send_response(HTTPStatus.OK)
        self.send_header(
            "Content-Type",
            f"multipart/x-mixed-replace; boundary={STREAM_BOUNDARY}",
        )
        self.send_header(
            "Cache-Control",
            "no-store, no-cache, must-revalidate, pre-check=0, post-check=0, max-age=0",
        )
        self.send_header("Pragma", "no-cache")
        self.send_header("Connection", "close")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()

        try:
            self._write_part(jpeg)
            while True:
                next_jpeg, seq, available = buf.wait_for_next(seq, timeout=2.0)
                if not available:
                    break
                if next_jpeg is None:
                    # Keep the connection alive and re-send the latest frame if idle.
                    latest = buf.latest_jpeg
                    if not latest:
                        break
                    self._write_part(latest)
                else:
                    self._write_part(next_jpeg)
        except (BrokenPipeError, ConnectionResetError, OSError):
            return

    def _write_part(self, jpeg: bytes) -> None:
        header = (
            f"--{STREAM_BOUNDARY}\r\n"
            "Content-Type: image/jpeg\r\n"
            f"Content-Length: {len(jpeg)}\r\n\r\n"
        ).encode("ascii")
        self.wfile.write(header)
        self.wfile.write(jpeg)
        self.wfile.write(b"\r\n")
        self.wfile.flush()

    def _send_json(self, status: HTTPStatus, payload: dict) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.end_headers()
        self.wfile.write(body)


class MjpegStreamServer:
    """Lifecycle wrapper around ``ThreadingHTTPServer`` for ``CameraNode``."""

    def __init__(
        self,
        frame_buffer: Optional[MjpegFrameBuffer] = None,
        host: str = DEFAULT_STREAM_HOST,
        port: int = DEFAULT_STREAM_PORT,
    ) -> None:
        self.frame_buffer = frame_buffer or MjpegFrameBuffer()
        self.host = host
        self.port = int(port)
        self._httpd: Optional[ThreadingHTTPServer] = None
        self._thread: Optional[threading.Thread] = None

    @property
    def server_port(self) -> int:
        if self._httpd is not None:
            return int(self._httpd.server_address[1])
        return self.port

    def start(self) -> bool:
        if self._httpd is not None:
            return True
        try:
            httpd = ThreadingHTTPServer((self.host, self.port), _MjpegRequestHandler)
            httpd.daemon_threads = True
            httpd.frame_buffer = self.frame_buffer  # type: ignore[attr-defined]
            self._httpd = httpd
            self._thread = threading.Thread(
                target=httpd.serve_forever,
                name="oak-mjpeg-http",
                daemon=True,
            )
            self._thread.start()
            return True
        except OSError:
            self._httpd = None
            self._thread = None
            return False

    def stop(self) -> None:
        self.frame_buffer.close()
        httpd = self._httpd
        self._httpd = None
        if httpd is not None:
            try:
                httpd.shutdown()
                httpd.server_close()
            except Exception:
                pass
        thread = self._thread
        self._thread = None
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
