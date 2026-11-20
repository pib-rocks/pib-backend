"""Unit tests for rosbridge + rosapi supervision and liveness watchdog (PR-2177)."""

from __future__ import annotations

import base64
import hashlib
import importlib.util
import json
import signal
import socket
import struct
import threading
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
RUN_ROSBRIDGE = REPO_ROOT / "ros_packages" / "rosbridge" / "run_rosbridge.py"
ROSBRIDGE_DOCKERFILE = REPO_ROOT / "ros_packages" / "rosbridge" / "Dockerfile"
COMPOSE_FILE = REPO_ROOT / "docker-compose.yaml"


import sys


def _load_module():
    spec = importlib.util.spec_from_file_location("pib_run_rosbridge", RUN_ROSBRIDGE)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _start_fake_ws_server(reply_mode: str):
    """Minimal RFC6455 server on an ephemeral localhost port."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", 0))
    srv.listen(4)
    port = srv.getsockname()[1]
    stop = threading.Event()

    def _serve_one(conn: socket.socket) -> None:
        try:
            conn.settimeout(2.0)
            req = b""
            while b"\r\n\r\n" not in req:
                chunk = conn.recv(4096)
                if not chunk:
                    return
                req += chunk
            key = ""
            for line in req.decode("latin1").splitlines():
                if line.lower().startswith("sec-websocket-key:"):
                    key = line.split(":", 1)[1].strip()
            accept = base64.b64encode(
                hashlib.sha1(
                    (key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode("ascii")
                ).digest()
            ).decode("ascii")
            conn.sendall(
                (
                    "HTTP/1.1 101 Switching Protocols\r\n"
                    "Upgrade: websocket\r\n"
                    "Connection: Upgrade\r\n"
                    f"Sec-WebSocket-Accept: {accept}\r\n\r\n"
                ).encode("ascii")
            )
            # Read client masked frame
            hdr = conn.recv(2)
            if len(hdr) < 2:
                return
            length = hdr[1] & 0x7F
            if length == 126:
                length = struct.unpack("!H", conn.recv(2))[0]
            mask = conn.recv(4)
            raw = b""
            while len(raw) < length:
                raw += conn.recv(length - len(raw))
            msg = json.loads(bytes(b ^ mask[i & 3] for i, b in enumerate(raw)))
            if reply_mode == "hang":
                stop.wait(1.5)
                return
            if reply_mode == "ok":
                body = json.dumps(
                    {
                        "op": "service_response",
                        "id": msg["id"],
                        "service": "/rosapi/nodes",
                        "result": True,
                        "values": {"nodes": ["/rosapi", "/rosbridge_websocket"]},
                    }
                ).encode("utf-8")
            else:
                body = json.dumps(
                    {
                        "op": "service_response",
                        "id": msg["id"],
                        "service": "/rosapi/nodes",
                        "result": False,
                        "values": "Timeout exceeded",
                    }
                ).encode("utf-8")
            frame = bytearray([0x81])
            if len(body) < 126:
                frame.append(len(body))
            else:
                frame.append(126)
                frame.extend(struct.pack("!H", len(body)))
            frame.extend(body)
            conn.sendall(bytes(frame))
        finally:
            conn.close()

    def _accept():
        srv.settimeout(0.2)
        while not stop.is_set():
            try:
                conn, _ = srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            threading.Thread(target=_serve_one, args=(conn,), daemon=True).start()

    thread = threading.Thread(target=_accept, daemon=True)
    thread.start()
    return srv, port, stop


def test_compose_and_dockerfile_wire_supervised_rosbridge():
    dockerfile = ROSBRIDGE_DOCKERFILE.read_text(encoding="utf-8")
    assert "COPY ./rosbridge/run_rosbridge.py /opt/pib/run_rosbridge.py" in dockerfile

    compose = yaml.safe_load(COMPOSE_FILE.read_text(encoding="utf-8"))
    cmd = compose["services"]["rosbridge-ws"]["command"]
    assert "/opt/pib/run_rosbridge.py" in cmd

    mod = _load_module()
    launch_cmd = mod.build_launch_command()
    assert "respawn:=true" in launch_cmd
    assert "default_call_service_timeout:=90.0" in launch_cmd


def test_probe_rosapi_succeeds_against_live_mock_ws():
    mod = _load_module()
    srv, port, stop = _start_fake_ws_server("ok")
    try:
        status, _detail = mod.probe_rosapi("127.0.0.1", port, timeout_sec=2.0)
        assert status == mod.PROBE_OK
    finally:
        stop.set()
        srv.close()


def test_probe_rosapi_detects_hung_service_without_reporting_ws_down():
    mod = _load_module()
    srv, port, stop = _start_fake_ws_server("hang")
    try:
        status, _detail = mod.probe_rosapi("127.0.0.1", port, timeout_sec=0.4)
        assert status == mod.PROBE_ROSAPI_HUNG
    finally:
        stop.set()
        srv.close()


def test_watchdog_terminates_only_rosapi_after_consecutive_hangs():
    mod = _load_module()
    rosapi_kills = 0
    launch_stops = 0

    def fake_kill():
        nonlocal rosapi_kills
        rosapi_kills += 1
        return [4242]

    def fake_stop():
        nonlocal launch_stops
        launch_stops += 1

    responses = iter(
        [
            (mod.PROBE_ROSAPI_HUNG, "timeout 1"),
            (mod.PROBE_OK, "recovered"),
            (mod.PROBE_ROSAPI_HUNG, "timeout 1"),
            (mod.PROBE_ROSAPI_HUNG, "timeout 2"),
            (mod.PROBE_ROSAPI_HUNG, "timeout 3"),
        ]
    )
    wd = mod.RosapiWatchdog(
        config=mod.WatchdogConfig(max_failures=3),
        probe_fn=lambda _h, _p, _t: next(responses),
        kill_rosapi_fn=fake_kill,
        stop_launch_fn=fake_stop,
    )

    assert wd.step() == mod.PROBE_ROSAPI_HUNG
    assert wd.step() == mod.PROBE_OK  # resets counter
    assert rosapi_kills == 0

    assert wd.step() == mod.PROBE_ROSAPI_HUNG
    assert wd.step() == mod.PROBE_ROSAPI_HUNG
    assert rosapi_kills == 0
    assert wd.step() == mod.PROBE_ROSAPI_HUNG
    assert rosapi_kills == 1
    assert launch_stops == 0


def test_watchdog_stops_container_when_websocket_itself_is_down():
    mod = _load_module()
    rosapi_kills = 0
    launch_stops = 0

    wd = mod.RosapiWatchdog(
        config=mod.WatchdogConfig(max_failures=2),
        probe_fn=lambda _h, _p, _t: (mod.PROBE_WS_DOWN, "connection refused"),
        kill_rosapi_fn=lambda: [1],
        stop_launch_fn=lambda: None,
    )
    wd.kill_rosapi_fn = lambda: (rosapi_kills := rosapi_kills + 1) or []  # type: ignore[assignment]

    def _stop():
        nonlocal launch_stops
        launch_stops += 1

    wd.stop_launch_fn = _stop

    wd.step()
    assert launch_stops == 0
    wd.step()
    assert launch_stops == 1
    assert wd.rosapi_restarts == 0


def test_find_and_terminate_rosapi_pids_targets_only_rosapi_node(tmp_path: Path):
    mod = _load_module()
    (tmp_path / "101").mkdir()
    (tmp_path / "101" / "cmdline").write_bytes(
        b"/usr/bin/python3\x00/opt/ros/jazzy/lib/rosapi/rosapi_node\x00"
    )
    (tmp_path / "102").mkdir()
    (tmp_path / "102" / "cmdline").write_bytes(
        b"/usr/bin/python3\x00/opt/pib/run_rosbridge.py\x00"
    )
    (tmp_path / "103").mkdir()
    (tmp_path / "103" / "cmdline").write_bytes(
        b"/usr/bin/python3\x00/opt/ros/jazzy/lib/rosbridge_server/rosbridge_websocket\x00"
    )

    assert mod.find_rosapi_pids(tmp_path) == [101]

    sent: list[tuple[int, int]] = []

    def fake_kill(pid: int, sig: int) -> None:
        sent.append((pid, sig))
        if sig == signal.SIGTERM:
            # Remove fake proc dir to simulate clean exit
            (tmp_path / str(pid) / "cmdline").unlink()
            (tmp_path / str(pid)).rmdir()

    killed = mod.terminate_rosapi_node(
        proc_root=tmp_path, kill_fn=fake_kill, sleep_fn=lambda _s: None, grace_sec=1.0
    )
    assert killed == [101]
    assert sent == [(101, signal.SIGTERM)]
