#!/usr/bin/env python3
"""Supervise rosbridge_websocket + rosapi_node with active rosapi liveness recovery (PR-2177).

Launches ``rosbridge_websocket_launch.xml`` with ``respawn:=true`` so a crashed
``rosapi_node`` or ``rosbridge_websocket`` restarts immediately, and runs a
lightweight stdlib-only WebSocket watchdog probing ``/rosapi/nodes`` over
``127.0.0.1:9090``.

- If ``rosapi_node`` hangs (WebSocket accepts connections, ``/rosapi/nodes``
  times out for ``max_failures`` consecutive probes), only ``rosapi_node`` is
  terminated so ``ros2 launch`` respawns it without dropping WebSocket clients.
- If ``rosbridge_websocket`` itself stops accepting connections for
  ``max_failures`` consecutive probes, this supervisor stops the launch process
  and exits non-zero so Docker's ``restart: always`` recreates the bridge.
"""

from __future__ import annotations

import base64
import hashlib
import json
import logging
import os
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable

LOG = logging.getLogger("rosbridge_watchdog")

PROBE_OK = "ok"
PROBE_WS_DOWN = "ws_down"
PROBE_ROSAPI_HUNG = "rosapi_hung"


def _ws_handshake(sock: socket.socket, host: str, port: int) -> None:
    key = base64.b64encode(os.urandom(16)).decode("ascii")
    request = (
        f"GET / HTTP/1.1\r\n"
        f"Host: {host}:{port}\r\n"
        f"Upgrade: websocket\r\n"
        f"Connection: Upgrade\r\n"
        f"Sec-WebSocket-Key: {key}\r\n"
        f"Sec-WebSocket-Version: 13\r\n\r\n"
    )
    sock.sendall(request.encode("ascii"))
    buf = b""
    while b"\r\n\r\n" not in buf:
        chunk = sock.recv(4096)
        if not chunk:
            raise ConnectionError("EOF during WebSocket handshake")
        buf += chunk
        if len(buf) > 16384:
            raise ConnectionError("WebSocket handshake header too large")
    header = buf.split(b"\r\n\r\n", 1)[0].decode("latin1")
    first_line = header.splitlines()[0] if header else ""
    if " 101 " not in first_line:
        raise ConnectionError(f"WebSocket upgrade rejected: {first_line!r}")
    expected = base64.b64encode(
        hashlib.sha1(
            (key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode("ascii")
        ).digest()
    ).decode("ascii")
    if expected not in header:
        raise ConnectionError("WebSocket Sec-WebSocket-Accept mismatch")


def _ws_send_text(sock: socket.socket, text: str) -> None:
    payload = text.encode("utf-8")
    mask = os.urandom(4)
    length = len(payload)
    header = bytearray([0x81])  # FIN + text frame
    if length < 126:
        header.append(0x80 | length)
    elif length < 65536:
        header.append(0x80 | 126)
        header.extend(struct.pack("!H", length))
    else:
        header.append(0x80 | 127)
        header.extend(struct.pack("!Q", length))
    header.extend(mask)
    masked = bytes(b ^ mask[i & 3] for i, b in enumerate(payload))
    sock.sendall(bytes(header) + masked)


def _recv_exact(sock: socket.socket, count: int) -> bytes:
    out = bytearray()
    while len(out) < count:
        chunk = sock.recv(count - len(out))
        if not chunk:
            raise ConnectionError("EOF while reading WebSocket frame")
        out.extend(chunk)
    return bytes(out)


def _ws_recv_text(sock: socket.socket, deadline: float) -> str:
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("Timed out waiting for WebSocket frame")
        sock.settimeout(remaining)
        b1, b2 = _recv_exact(sock, 2)
        opcode = b1 & 0x0F
        masked = bool(b2 & 0x80)
        length = b2 & 0x7F
        if length == 126:
            length = struct.unpack("!H", _recv_exact(sock, 2))[0]
        elif length == 127:
            length = struct.unpack("!Q", _recv_exact(sock, 8))[0]
        mask = _recv_exact(sock, 4) if masked else b""
        payload = _recv_exact(sock, length) if length else b""
        if masked:
            payload = bytes(b ^ mask[i & 3] for i, b in enumerate(payload))
        if opcode == 0x8:  # close
            raise ConnectionError("WebSocket closed by server")
        if opcode == 0x9:  # ping -> pong
            pong = bytearray([0x8A, 0x80 | min(len(payload), 125)])
            pong_mask = os.urandom(4)
            pong.extend(pong_mask)
            pong.extend(
                bytes(b ^ pong_mask[i & 3] for i, b in enumerate(payload[:125]))
            )
            sock.sendall(bytes(pong))
            continue
        if opcode == 0x1:  # text
            return payload.decode("utf-8", errors="replace")


def probe_rosapi(
    host: str = "127.0.0.1",
    port: int = 9090,
    timeout_sec: float = 10.0,
) -> tuple[str, str]:
    """Probe ``/rosapi/nodes`` over rosbridge WebSocket.

    Returns ``(PROBE_OK | PROBE_WS_DOWN | PROBE_ROSAPI_HUNG, detail)``.
    """
    call_id = f"rosapi_watchdog:{int(time.monotonic() * 1000)}"
    deadline = time.monotonic() + timeout_sec
    try:
        sock = socket.create_connection((host, port), timeout=min(timeout_sec, 5.0))
    except OSError as exc:
        return PROBE_WS_DOWN, f"connect failed: {exc}"

    try:
        sock.settimeout(max(0.5, deadline - time.monotonic()))
        _ws_handshake(sock, host, port)
    except Exception as exc:  # noqa: BLE001
        sock.close()
        return PROBE_WS_DOWN, f"handshake failed: {exc}"

    try:
        req = {
            "op": "call_service",
            "id": call_id,
            "service": "/rosapi/nodes",
            "args": {},
        }
        _ws_send_text(sock, json.dumps(req))
        while True:
            raw = _ws_recv_text(sock, deadline)
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                continue
            if msg.get("op") != "service_response":
                continue
            if msg.get("id") != call_id:
                continue
            if msg.get("result") is True:
                return PROBE_OK, "rosapi answered /rosapi/nodes"
            return (
                PROBE_ROSAPI_HUNG,
                f"/rosapi/nodes returned result={msg.get('result')!r}: {msg.get('values')!r}",
            )
    except Exception as exc:  # noqa: BLE001
        return PROBE_ROSAPI_HUNG, f"/rosapi/nodes probe failed: {exc}"
    finally:
        try:
            sock.close()
        except OSError:
            pass


def find_rosapi_pids(proc_root: Path = Path("/proc")) -> list[int]:
    """Return PIDs whose cmdline invokes ``rosapi_node`` (excluding ourselves)."""
    self_pid = os.getpid()
    pids: list[int] = []
    if not proc_root.is_dir():
        return pids
    for entry in proc_root.iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        if pid == self_pid:
            continue
        try:
            cmdline = (entry / "cmdline").read_bytes().decode("utf-8", errors="ignore")
        except OSError:
            continue
        if "rosapi_node" in cmdline and "run_rosbridge.py" not in cmdline:
            pids.append(pid)
    return sorted(pids)


def terminate_rosapi_node(
    proc_root: Path = Path("/proc"),
    kill_fn: Callable[[int, int], None] = os.kill,
    sleep_fn: Callable[[float], None] = time.sleep,
    grace_sec: float = 5.0,
) -> list[int]:
    """Terminate hung ``rosapi_node`` process(es) so ``ros2 launch`` respawns it."""
    pids = find_rosapi_pids(proc_root)
    if not pids:
        LOG.warning(
            "rosapi_node PID not found in %s; waiting for launch respawn", proc_root
        )
        return []
    for pid in pids:
        try:
            kill_fn(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    deadline = time.monotonic() + grace_sec
    while time.monotonic() < deadline:
        alive = [p for p in pids if (proc_root / str(p)).exists()]
        if not alive:
            return pids
        sleep_fn(0.2)
    for pid in pids:
        if (proc_root / str(pid)).exists():
            try:
                kill_fn(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
    return pids


@dataclass
class WatchdogConfig:
    interval_sec: float = 30.0
    timeout_sec: float = 10.0
    startup_grace_sec: float = 30.0
    max_failures: int = 3
    host: str = "127.0.0.1"
    port: int = 9090


class RosapiWatchdog:
    """State machine for periodic rosapi + rosbridge liveness checks."""

    def __init__(
        self,
        config: WatchdogConfig,
        probe_fn: Callable[[str, int, float], tuple[str, str]] = probe_rosapi,
        kill_rosapi_fn: Callable[[], Iterable[int]] = terminate_rosapi_node,
        stop_launch_fn: Callable[[], None] = lambda: None,
    ) -> None:
        self.config = config
        self.probe_fn = probe_fn
        self.kill_rosapi_fn = kill_rosapi_fn
        self.stop_launch_fn = stop_launch_fn
        self.rosapi_failures = 0
        self.ws_failures = 0
        self.rosapi_restarts = 0
        self.launch_stops = 0

    def step(self) -> str:
        status, detail = self.probe_fn(
            self.config.host, self.config.port, self.config.timeout_sec
        )
        if status == PROBE_OK:
            if self.rosapi_failures or self.ws_failures:
                LOG.info("rosapi watchdog recovered: %s", detail)
            self.rosapi_failures = 0
            self.ws_failures = 0
            return PROBE_OK

        if status == PROBE_ROSAPI_HUNG:
            self.ws_failures = 0
            self.rosapi_failures += 1
            LOG.warning(
                "rosapi liveness check failed (%d/%d): %s",
                self.rosapi_failures,
                self.config.max_failures,
                detail,
            )
            if self.rosapi_failures >= self.config.max_failures:
                LOG.error(
                    "rosapi hung for %d consecutive probes; terminating rosapi_node for respawn",
                    self.rosapi_failures,
                )
                self.kill_rosapi_fn()
                self.rosapi_restarts += 1
                self.rosapi_failures = 0
            return PROBE_ROSAPI_HUNG

        self.rosapi_failures = 0
        self.ws_failures += 1
        LOG.warning(
            "rosbridge websocket probe failed (%d/%d): %s",
            self.ws_failures,
            self.config.max_failures,
            detail,
        )
        if self.ws_failures >= self.config.max_failures:
            LOG.error(
                "rosbridge websocket unreachable for %d consecutive probes; stopping container for restart",
                self.ws_failures,
            )
            self.stop_launch_fn()
            self.launch_stops += 1
            self.ws_failures = 0
        return PROBE_WS_DOWN


def _env_float(name: str, default: float) -> float:
    raw = os.environ.get(name, "").strip()
    if not raw:
        return default
    try:
        return max(0.1, float(raw))
    except ValueError:
        return default


def _env_int(name: str, default: int) -> int:
    raw = os.environ.get(name, "").strip()
    if not raw:
        return default
    try:
        return max(1, int(raw))
    except ValueError:
        return default


def build_launch_command(extra_args: list[str] | None = None) -> list[str]:
    cmd = [
        "ros2",
        "launch",
        "rosbridge_server",
        "rosbridge_websocket_launch.xml",
        f"default_call_service_timeout:={os.environ.get('ROSBRIDGE_CALL_SERVICE_TIMEOUT', '90.0')}",
        "respawn:=true",
    ]
    if extra_args:
        cmd.extend(extra_args)
    return cmd


def main(argv: list[str] | None = None) -> int:
    logging.basicConfig(
        level=logging.INFO,
        format="[rosbridge_watchdog] [%(levelname)s] %(message)s",
    )
    args = list(sys.argv[1:] if argv is None else argv)
    cmd = build_launch_command(args)
    LOG.info("Starting rosbridge stack: %s", " ".join(cmd))
    proc = subprocess.Popen(cmd)
    stop_event = threading.Event()
    exit_code_override: list[int] = []

    def _stop_launch(force_nonzero: bool = False) -> None:
        if force_nonzero:
            exit_code_override.append(1)
        stop_event.set()
        if proc.poll() is None:
            proc.terminate()

    def _handle_signal(signum: int, _frame: object) -> None:
        LOG.info("Received signal %d; forwarding to ros2 launch", signum)
        _stop_launch(force_nonzero=False)

    signal.signal(signal.SIGTERM, _handle_signal)
    signal.signal(signal.SIGINT, _handle_signal)

    config = WatchdogConfig(
        interval_sec=_env_float("ROSAPI_WATCHDOG_INTERVAL_SEC", 30.0),
        timeout_sec=_env_float("ROSAPI_WATCHDOG_TIMEOUT_SEC", 10.0),
        startup_grace_sec=_env_float("ROSAPI_WATCHDOG_STARTUP_GRACE_SEC", 30.0),
        max_failures=_env_int("ROSAPI_WATCHDOG_MAX_FAILURES", 3),
        port=_env_int("ROSBRIDGE_PORT", 9090),
    )
    watchdog = RosapiWatchdog(
        config=config,
        stop_launch_fn=lambda: _stop_launch(force_nonzero=True),
    )

    def _watchdog_loop() -> None:
        if stop_event.wait(config.startup_grace_sec):
            return
        while not stop_event.is_set() and proc.poll() is None:
            watchdog.step()
            if stop_event.wait(config.interval_sec):
                break

    watcher = threading.Thread(
        target=_watchdog_loop, name="rosapi-watchdog", daemon=True
    )
    watcher.start()

    try:
        rc = proc.wait()
    finally:
        stop_event.set()
        if proc.poll() is None:
            proc.kill()
            proc.wait()
    return exit_code_override[0] if exit_code_override else rc


if __name__ == "__main__":
    raise SystemExit(main())
