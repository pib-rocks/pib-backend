#!/usr/bin/env python3
"""Sequential, non-actuating live verification of the on-device model registry."""

from __future__ import annotations

import argparse
import json
import os
import shlex
import socket
import subprocess
import sys
import time
import uuid
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

STATUS_TYPE = "datatypes/msg/ModelStatusArray"
DETECTION_TYPE = "datatypes/msg/DetectionArray"
LIST_TYPE = "datatypes/srv/ListModels"
START_TYPE = "datatypes/srv/StartModel"
STOP_TYPE = "datatypes/srv/StopModel"
LOG_MARKERS = (
    "crash",
    "x_link_error",
    "reconnect",
    "re-connecting",
    "traceback",
    "segmentation fault",
    "core dumped",
    "fatal",
    "process has died",
)
STAGE_MARKER = "stage packets total:"
CAMERA_STATE_MARKER = "stereo depth available"


@dataclass
class EntryResult:
    model_id: str
    declared_shaves: int
    available: bool
    start_success: bool = False
    start_message: str = ""
    state_transitions: list[dict[str, Any]] = field(default_factory=list)
    final_state: str = "unobserved"
    active: bool = False
    measured_fps: float = 0.0
    published_messages: int = 0
    stage_counter_lines: list[str] = field(default_factory=list)
    camera_state_lines: list[str] = field(default_factory=list)
    warning_lines: list[str] = field(default_factory=list)
    log_since: str = ""
    log_error: str = ""
    stop_success: bool = False
    stop_message: str = ""
    shave_basis: str = "unconfirmed"
    interrupted: bool = False
    error: str = ""


class RosbridgeClient:
    def __init__(self, connection: Any):
        self.connection = connection
        self.pending: list[dict[str, Any]] = []
        self.observers: list[Callable[[dict[str, Any]], None]] = []

    def send(self, message: dict[str, Any]) -> None:
        self.connection.send(json.dumps(message))

    def subscribe(self, topic: str, message_type: str) -> None:
        self.send(
            {
                "op": "subscribe",
                "id": f"model-measure-sub-{uuid.uuid4()}",
                "topic": topic,
                "type": message_type,
            }
        )

    def _receive(self, timeout: float) -> dict[str, Any] | None:
        try:
            self.connection.settimeout(timeout)
            raw = self.connection.recv()
        except Exception as exc:
            if isinstance(exc, (TimeoutError, socket.timeout)) or (
                exc.__class__.__name__ == "WebSocketTimeoutException"
            ):
                return None
            raise
        if not raw:
            return None
        message = json.loads(raw)
        for observer in self.observers:
            observer(message)
        return message

    def receive_matching(
        self, predicate: Callable[[dict[str, Any]], bool], timeout: float
    ) -> dict[str, Any] | None:
        for index, message in enumerate(self.pending):
            if predicate(message):
                return self.pending.pop(index)

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            message = self._receive(min(1.0, deadline - time.monotonic()))
            if message is None:
                continue
            if predicate(message):
                return message
            self.pending.append(message)
        return None

    def pump(self, duration: float) -> None:
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            message = self._receive(min(0.5, deadline - time.monotonic()))
            if message is not None:
                self.pending.append(message)

    def call_service(
        self,
        service: str,
        service_type: str,
        args: dict[str, Any],
        timeout: float,
    ) -> dict[str, Any]:
        request_id = f"model-measure-{uuid.uuid4()}"
        self.send(
            {
                "op": "call_service",
                "id": request_id,
                "service": service,
                "type": service_type,
                "args": args,
            }
        )
        response = self.receive_matching(
            lambda message: message.get("op") == "service_response"
            and message.get("id") == request_id,
            timeout,
        )
        if response is None:
            raise TimeoutError(f"no response from {service} within {timeout}s")
        if response.get("result") is not True:
            raise RuntimeError(
                f"{service} transport failed: {response.get('values', response)}"
            )
        return response.get("values", {})


class EntryObserver:
    def __init__(self, model_id: str, started_at: float):
        self.model_id = model_id
        self.started_at = started_at
        self.detection_count = 0
        self.transitions: list[dict[str, Any]] = []
        self.latest_status: dict[str, Any] | None = None
        self.max_fps = 0.0

    def __call__(self, message: dict[str, Any]) -> None:
        if message.get("op") != "publish":
            return
        topic = message.get("topic")
        if topic == f"/detections/{self.model_id}":
            self.detection_count += 1
            return
        if topic != "/models_status":
            return
        status = next(
            (
                item
                for item in message.get("msg", {}).get("models", [])
                if item.get("model_id") == self.model_id
            ),
            None,
        )
        if status is None:
            return
        self.latest_status = status
        fps = float(status.get("fps", 0.0))
        self.max_fps = max(self.max_fps, fps)
        sample = {
            "elapsed_s": round(time.monotonic() - self.started_at, 3),
            "state": status.get("state", ""),
            "active": bool(status.get("active", False)),
            "fps": fps,
            "shaves": int(status.get("shaves", 0)),
            "message": status.get("message", ""),
        }
        transition_key = (sample["state"], sample["active"], sample["message"])
        if not self.transitions or transition_key != (
            self.transitions[-1]["state"],
            self.transitions[-1]["active"],
            self.transitions[-1]["message"],
        ):
            self.transitions.append(sample)


def _ssh_base(args: argparse.Namespace) -> list[str]:
    command = [
        "ssh",
        "-o",
        "BatchMode=yes" if not args.ssh_password else "BatchMode=no",
        "-o",
        "ConnectTimeout=25",
        "-o",
        "StrictHostKeyChecking=no",
        f"{args.ssh_user}@{args.host}",
    ]
    if args.ssh_password:
        command = ["sshpass", "-p", args.ssh_password, *command]
    return command


def _remote_command(args: argparse.Namespace, command: str) -> str:
    completed = subprocess.run(
        [*_ssh_base(args), command],
        check=False,
        capture_output=True,
        text=True,
        timeout=args.service_timeout,
    )
    if completed.returncode:
        detail = completed.stderr.strip() or completed.stdout.strip()
        raise RuntimeError(f"remote command failed ({completed.returncode}): {detail}")
    return completed.stdout


def _remote_epoch(args: argparse.Namespace) -> str:
    return _remote_command(args, "date +%s.%N").strip()


def _camera_logs(args: argparse.Namespace, since: str) -> str:
    container = shlex.quote(args.camera_container)
    return _remote_command(
        args,
        f"docker logs --timestamps --since {shlex.quote(since)} {container} 2>&1",
    )


def _log_evidence(logs: str) -> tuple[list[str], list[str], list[str]]:
    lines = [line.strip() for line in logs.splitlines()]
    stages = [line for line in lines if STAGE_MARKER in line.lower()]
    camera_state = [line for line in lines if CAMERA_STATE_MARKER in line.lower()]
    warnings = [
        line for line in lines if any(marker in line.lower() for marker in LOG_MARKERS)
    ]
    return stages, camera_state, warnings


def _observe_until_terminal(
    ros: RosbridgeClient, observer: EntryObserver, timeout: float
) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if observer.latest_status is not None and observer.latest_status.get(
            "state"
        ) in {"running", "failed"}:
            return
        ros.pump(min(0.5, deadline - time.monotonic()))


def _format_result(result: EntryResult) -> str:
    return "RESULT " + json.dumps(asdict(result), separators=(",", ":"))


def _stop_model(
    ros: RosbridgeClient,
    model_id: str,
    owner: str,
    timeout: float,
) -> tuple[bool, str, bool]:
    errors = []
    interrupted = False
    for _attempt in range(2):
        try:
            stopped = ros.call_service(
                "/stop_model",
                STOP_TYPE,
                {"model_id": model_id, "owner": owner},
                timeout,
            )
            return (
                bool(stopped.get("success", False)),
                str(stopped.get("message", "")),
                interrupted,
            )
        except BaseException as exc:
            interrupted = interrupted or isinstance(
                exc, (KeyboardInterrupt, SystemExit)
            )
            errors.append(f"{type(exc).__name__}: {exc}")
    return False, "; ".join(errors), interrupted


def measure_entry(
    ros: RosbridgeClient,
    model: dict[str, Any],
    args: argparse.Namespace,
    owner: str,
) -> EntryResult:
    result = EntryResult(
        model_id=model["model_id"],
        declared_shaves=int(model["shaves"]),
        available=bool(model["available"]),
    )
    started_at = time.monotonic()
    observer = EntryObserver(result.model_id, started_at)
    log_since = ""
    try:
        log_since = _remote_epoch(args)
        result.log_since = log_since
    except Exception as exc:
        result.log_error = str(exc)

    ros.subscribe(f"/detections/{result.model_id}", DETECTION_TYPE)
    ros.observers.append(observer)
    try:
        started = ros.call_service(
            "/start_model",
            START_TYPE,
            {
                "model_id": result.model_id,
                "shaves": result.declared_shaves,
                "owner": owner,
            },
            args.service_timeout,
        )
        result.start_success = bool(started.get("success", False))
        result.start_message = str(started.get("message", ""))
        _observe_until_terminal(ros, observer, args.status_timeout)
        if result.start_success:
            observer.detection_count = 0
            ros.pump(args.measure_seconds)
    except BaseException as exc:
        result.interrupted = isinstance(exc, (KeyboardInterrupt, SystemExit))
        result.error = f"{type(exc).__name__}: {exc}"
    finally:
        result.published_messages = observer.detection_count
        result.state_transitions = list(observer.transitions)
        result.measured_fps = round(observer.max_fps, 3)
        if observer.latest_status is not None:
            result.final_state = str(observer.latest_status.get("state", ""))
            result.active = bool(observer.latest_status.get("active", False))

        stop_success, stop_message, stop_interrupted = _stop_model(
            ros,
            result.model_id,
            owner,
            args.service_timeout,
        )
        result.stop_success = stop_success
        result.stop_message = stop_message
        result.interrupted = result.interrupted or stop_interrupted
        if not result.stop_success:
            release_error = f"owner release unconfirmed: {result.stop_message}"
            result.error = "; ".join(filter(None, (result.error, release_error)))

        if log_since:
            try:
                logs = _camera_logs(args, log_since)
                (
                    result.stage_counter_lines,
                    result.camera_state_lines,
                    result.warning_lines,
                ) = _log_evidence(logs)
            except Exception as exc:
                result.log_error = str(exc)
        ros.observers.remove(observer)

    status_shaves = (
        int(observer.latest_status.get("shaves", -1))
        if observer.latest_status is not None
        else -1
    )
    if (
        result.start_success
        and result.final_state == "running"
        and result.active
        and result.measured_fps > 0
        and status_shaves == result.declared_shaves
    ):
        result.shave_basis = (
            f"confirmed: requested={result.declared_shaves}, "
            f"status={status_shaves}, fps={result.measured_fps}"
        )
    else:
        result.shave_basis = (
            f"unconfirmed: requested={result.declared_shaves}, "
            f"status={status_shaves}, fps={result.measured_fps}"
        )
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.1.92")
    parser.add_argument("--rosbridge-url")
    parser.add_argument("--ssh-user", default="pib")
    parser.add_argument(
        "--ssh-password",
        default=os.environ.get("PIB_ROBOT_SSH_PASSWORD", ""),
        help="prefer PIB_ROBOT_SSH_PASSWORD to exposing this in shell history",
    )
    parser.add_argument("--camera-container", default="multirepo-ros-camera-1")
    parser.add_argument("--service-timeout", type=float, default=120.0)
    parser.add_argument("--status-timeout", type=float, default=30.0)
    parser.add_argument("--measure-seconds", type=float, default=15.0)
    parser.add_argument(
        "--expected-count",
        type=int,
        default=13,
        help="abort before starting anything if /list_models differs; 0 disables",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("model-measurements.json"),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rosbridge_url = args.rosbridge_url or f"ws://{args.host}:9090"
    try:
        import websocket
    except ImportError:
        print("websocket-client is required", file=sys.stderr)
        return 2

    connection = websocket.create_connection(
        rosbridge_url, timeout=args.service_timeout
    )
    ros = RosbridgeClient(connection)
    run_id = uuid.uuid4().hex
    owner = f"model-measure-{run_id}"
    results: list[EntryResult] = []
    try:
        ros.subscribe("/models_status", STATUS_TYPE)
        listed = ros.call_service("/list_models", LIST_TYPE, {}, args.service_timeout)
        models = [model for model in listed.get("models", []) if model.get("available")]
        if args.expected_count and len(models) != args.expected_count:
            raise RuntimeError(
                f"expected {args.expected_count} available entries, got {len(models)}"
            )
        status_frame = ros.receive_matching(
            lambda message: message.get("op") == "publish"
            and message.get("topic") == "/models_status",
            5.0,
        )
        if status_frame is None:
            raise RuntimeError("no /models_status preflight snapshot")
        statuses = {
            status["model_id"]: status
            for status in status_frame.get("msg", {}).get("models", [])
        }
        contaminated = [
            model["model_id"]
            for model in models
            if model.get("active")
            or statuses.get(model["model_id"], {}).get("state") != "idle"
        ]
        if contaminated:
            raise RuntimeError(
                "refusing a contaminated sequential run; non-idle entries: "
                + ", ".join(contaminated)
            )
        print(
            f"RUN run_id={run_id} utc={datetime.now(timezone.utc).isoformat()} "
            f"rosbridge={rosbridge_url} available_entries={len(models)}",
            flush=True,
        )
        for model in models:
            result = measure_entry(ros, model, args, owner)
            results.append(result)
            print(_format_result(result), flush=True)
            if not result.stop_success or result.interrupted:
                reason = "interrupted" if result.interrupted else "release_unconfirmed"
                print(
                    f"ABORT model_id={result.model_id} reason={reason}",
                    flush=True,
                )
                break
    finally:
        connection.close()

    payload = {
        "run_id": run_id,
        "measured_at_utc": datetime.now(timezone.utc).isoformat(),
        "rosbridge_url": rosbridge_url,
        "camera_container": args.camera_container,
        "measure_seconds": args.measure_seconds,
        "results": [asdict(result) for result in results],
    }
    args.output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(f"WROTE {args.output} entries={len(results)}", flush=True)
    return 0 if results and all(result.stop_success for result in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
