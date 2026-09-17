"""Safe live-robot coverage for on-device models and their Blockly blocks.

The tests only load/unload models and record current motor telemetry as a pose.
They never invoke a pose, write a motor position, or send a trajectory.
"""

from __future__ import annotations

import json
import os
import socket
import time
import uuid
from dataclasses import dataclass, field
from typing import Any, Callable

import pytest
import requests

ROBOT_HOST = os.environ.get("PIB_MODEL_E2E_HOST", "192.168.1.92")
API_URL = os.environ.get("PIB_MODEL_E2E_API_URL", f"http://{ROBOT_HOST}:5000")
ROSBRIDGE_URL = os.environ.get("PIB_MODEL_E2E_ROSBRIDGE_URL", f"ws://{ROBOT_HOST}:9090")
REQUEST_TIMEOUT = 10
MODEL_TIMEOUT = 90
PROGRAM_TIMEOUT = 120


@dataclass
class RosbridgeClient:
    connection: Any
    pending: list[dict[str, Any]] = field(default_factory=list)

    def send(self, message: dict[str, Any]) -> None:
        self.connection.send(json.dumps(message))

    def receive_matching(
        self, predicate: Callable[[dict[str, Any]], bool], timeout: float
    ) -> dict[str, Any] | None:
        for index, message in enumerate(self.pending):
            if predicate(message):
                return self.pending.pop(index)

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                self.connection.settimeout(min(1.0, deadline - time.monotonic()))
                raw = self.connection.recv()
            except Exception as exc:
                if isinstance(exc, (TimeoutError, socket.timeout)) or (
                    exc.__class__.__name__ == "WebSocketTimeoutException"
                ):
                    continue
                raise
            if not raw:
                continue
            message = json.loads(raw)
            if predicate(message):
                return message
            self.pending.append(message)
        return None

    def call_service(
        self,
        service: str,
        service_type: str,
        args: dict[str, Any],
        timeout: float = REQUEST_TIMEOUT,
    ) -> dict[str, Any]:
        request_id = f"model-e2e-{uuid.uuid4()}"
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
            lambda message: message.get("id") == request_id, timeout
        )
        assert response is not None, f"no response from {service} within {timeout}s"
        assert (
            response.get("result") is True
        ), f"{service} transport failed: {response.get('values', response)}"
        return response.get("values", {})

    def subscribe(self, topic: str, message_type: str) -> None:
        self.send(
            {
                "op": "subscribe",
                "id": f"model-e2e-sub-{uuid.uuid4()}",
                "topic": topic,
                "type": message_type,
            }
        )

    def wait_for_topic(
        self,
        topic: str,
        timeout: float,
        predicate: Callable[[dict[str, Any]], bool] = lambda _message: True,
    ) -> dict[str, Any] | None:
        frame = self.receive_matching(
            lambda message: message.get("op") == "publish"
            and message.get("topic") == topic
            and predicate(message.get("msg", {})),
            timeout,
        )
        return None if frame is None else frame["msg"]


@pytest.fixture()
def live_robot():
    websocket = pytest.importorskip(
        "websocket", reason="websocket-client is required for the live model E2E"
    )
    session = requests.Session()
    try:
        response = session.get(f"{API_URL}/pose", timeout=REQUEST_TIMEOUT)
        response.raise_for_status()
        connection = websocket.create_connection(ROSBRIDGE_URL, timeout=REQUEST_TIMEOUT)
    except (requests.RequestException, OSError, websocket.WebSocketException) as exc:
        session.close()
        pytest.skip(f"live robot is unreachable at {ROBOT_HOST}: {exc}")

    client = RosbridgeClient(connection)
    try:
        yield session, client
    finally:
        connection.close()
        session.close()


def _assert_detection_array(message: dict[str, Any]) -> None:
    assert message["model_id"] == "hand_tracking"
    assert isinstance(message["frame_width"], int)
    assert isinstance(message["frame_height"], int)
    assert isinstance(message["detections"], list)
    for detection in message["detections"]:
        assert message["frame_width"] > 0
        assert message["frame_height"] > 0
        assert set(
            (
                "label",
                "score",
                "x_min",
                "y_min",
                "x_max",
                "y_max",
                "keypoint_names",
                "keypoint_x",
                "keypoint_y",
                "keypoint_z",
                "scalar_names",
                "scalar_values",
            )
        ).issubset(detection)
        assert len(detection["keypoint_names"]) == len(detection["keypoint_x"])
        assert len(detection["keypoint_names"]) == len(detection["keypoint_y"])
        assert len(detection["keypoint_names"]) == len(detection["keypoint_z"])
        assert len(detection["scalar_names"]) == len(detection["scalar_values"])
        assert 0 <= detection["score"] <= 1
        assert all(
            isinstance(detection[field], int)
            for field in ("x_min", "y_min", "x_max", "y_max")
        )
        assert all(value >= 0 for value in detection["keypoint_z"])


def _text(value: str) -> dict[str, Any]:
    return {"block": {"type": "text", "fields": {"TEXT": value}}}


def _number(value: int) -> dict[str, Any]:
    return {"block": {"type": "math_number", "fields": {"NUM": value}}}


def _blockly_pose_workspace(pose_name: str, marker: str) -> str:
    stop = {
        "block": {
            "type": "stop_model",
            "inputs": {"MODEL_ID": _text("hand_tracking")},
        }
    }
    print_marker = {
        "block": {
            "type": "text_print",
            "inputs": {"TEXT": _text(marker)},
            "next": stop,
        }
    }
    save_pose = {
        "block": {
            "type": "save_detection_as_pose",
            "inputs": {"NAME": _text(pose_name)},
            "next": print_marker,
        }
    }
    read_detection = {
        "block": {
            "type": "text_print",
            "inputs": {
                "TEXT": {
                    "block": {
                        "type": "get_detection_field",
                        "fields": {"FIELD": "label"},
                        "inputs": {
                            "MODEL_ID": _text("hand_tracking"),
                            "INDEX": _number(0),
                        },
                    }
                }
            },
            "next": save_pose,
        }
    }
    wait_for_hand = {
        "block": {
            "type": "sleep_for_seconds",
            "fields": {"SECONDS": 2},
            "next": read_detection,
        }
    }
    start = {
        "type": "start_model",
        "inputs": {
            "MODEL_ID": _text("hand_tracking"),
            "SHAVES": _number(0),
        },
        "next": wait_for_hand,
    }
    return json.dumps(
        {"blocks": {"languageVersion": 0, "blocks": [start]}},
        separators=(",", ":"),
    )


def test_model_lifecycle_and_detection_contract_e2e(live_robot):
    _session, ros = live_robot
    owner = f"pytest-e2e-{uuid.uuid4()}"
    ros.subscribe("/models_status", "datatypes/msg/ModelStatusArray")
    ros.subscribe("/detections/hand_tracking", "datatypes/msg/DetectionArray")

    listed = ros.call_service("/list_models", "datatypes/srv/ListModels", {})
    hand_model = next(
        model for model in listed["models"] if model["model_id"] == "hand_tracking"
    )
    assert set(
        (
            "model_id",
            "task",
            "licence",
            "shaves",
            "size_bytes",
            "available",
            "active",
        )
    ).issubset(hand_model)
    assert hand_model["available"] is True

    try:
        started = ros.call_service(
            "/start_model",
            "datatypes/srv/StartModel",
            {"model_id": "hand_tracking", "shaves": 0, "owner": owner},
            MODEL_TIMEOUT,
        )
        assert started["success"] is True, started["message"]

        status_array = ros.wait_for_topic(
            "/models_status",
            MODEL_TIMEOUT,
            lambda message: any(
                status.get("model_id") == "hand_tracking"
                and status.get("active") is True
                for status in message.get("models", [])
            ),
        )
        assert status_array is not None
        hand_status = next(
            status
            for status in status_array["models"]
            if status["model_id"] == "hand_tracking"
        )
        assert hand_status["state"] in {"starting", "running", "failed"}
        assert isinstance(hand_status["fps"], (int, float))
        assert hand_status["active"] is True

        service_detection = ros.call_service(
            "/get_detections",
            "datatypes/srv/GetDetections",
            {"model_id": "hand_tracking"},
        )["detections"]
        _assert_detection_array(service_detection)

        # A hand is not required in view. If the topic publishes, its payload must
        # obey the same contract; the Blockly test below covers the no-message path.
        topic_detection = ros.wait_for_topic("/detections/hand_tracking", timeout=5)
        if topic_detection is not None:
            _assert_detection_array(topic_detection)
    finally:
        stopped = ros.call_service(
            "/stop_model",
            "datatypes/srv/StopModel",
            {"model_id": "hand_tracking", "owner": owner},
            MODEL_TIMEOUT,
        )
        assert stopped["success"] is True, stopped["message"]


def test_blockly_detection_to_named_pose_e2e(live_robot):
    session, ros = live_robot
    unique = uuid.uuid4().hex
    pose_name = f"model-e2e-pose-{unique}"
    program_name = f"model-e2e-program-{unique}"
    marker = f"MODEL_E2E_COMPLETE_{unique}"
    program_number = None
    pose_id = None
    goal_id = None
    result = None

    ros.subscribe(
        "/proxy_run_program_feedback",
        "datatypes/msg/ProxyRunProgramFeedback",
    )
    ros.subscribe("/proxy_run_program_result", "datatypes/msg/ProxyRunProgramResult")

    try:
        created = session.post(
            f"{API_URL}/program",
            json={"name": program_name},
            timeout=REQUEST_TIMEOUT,
        )
        assert created.status_code == 201, created.text
        program_number = created.json()["programNumber"]

        workspace = _blockly_pose_workspace(pose_name, marker)
        compiled = session.put(
            f"{API_URL}/program/{program_number}/code",
            json={"codeVisual": workspace},
            timeout=REQUEST_TIMEOUT,
        )
        assert compiled.status_code == 200, compiled.text

        started = ros.call_service(
            "/proxy_run_program_start",
            "datatypes/srv/ProxyRunProgramStart",
            {"program_number": program_number},
        )
        goal_id = started["proxy_goal_id"]

        output_lines: list[dict[str, Any]] = []
        deadline = time.monotonic() + PROGRAM_TIMEOUT
        while time.monotonic() < deadline and result is None:
            feedback = ros.wait_for_topic(
                "/proxy_run_program_feedback",
                timeout=1,
                predicate=lambda message: message.get("proxy_goal_id") == goal_id,
            )
            if feedback is not None:
                output_lines.extend(feedback.get("output_lines", []))
            result = ros.wait_for_topic(
                "/proxy_run_program_result",
                timeout=1,
                predicate=lambda message: message.get("proxy_goal_id") == goal_id,
            )

        assert result is not None, "Blockly program did not finish"
        assert result["exit_code"] == 0
        output = "\n".join(line["content"] for line in output_lines)
        assert marker in output

        printed_lines = {line["content"].strip() for line in output_lines}
        no_detection_warning = (
            "no detection 0 received from model 'hand_tracking'" in output
        )
        if no_detection_warning:
            assert "0" in printed_lines
        else:
            assert "hand" in printed_lines, f"detection label was not printed: {output}"

        poses = session.get(f"{API_URL}/pose", timeout=REQUEST_TIMEOUT)
        poses.raise_for_status()
        saved_pose = next(
            pose for pose in poses.json()["poses"] if pose["name"] == pose_name
        )
        pose_id = saved_pose["poseId"]
    finally:
        if goal_id is not None and result is None:
            try:
                ros.call_service(
                    "/proxy_run_program_stop",
                    "datatypes/srv/ProxyRunProgramStop",
                    {"proxy_goal_id": goal_id},
                )
            except Exception:
                # Preserve the primary failure and continue deleting API records.
                pass
        if pose_id is None:
            try:
                poses = session.get(f"{API_URL}/pose", timeout=REQUEST_TIMEOUT)
                if poses.ok:
                    pose_id = next(
                        (
                            pose["poseId"]
                            for pose in poses.json()["poses"]
                            if pose["name"] == pose_name
                        ),
                        None,
                    )
            except requests.RequestException:
                pass
        if pose_id is not None:
            try:
                session.delete(f"{API_URL}/pose/{pose_id}", timeout=REQUEST_TIMEOUT)
            finally:
                if program_number is not None:
                    session.delete(
                        f"{API_URL}/program/{program_number}",
                        timeout=REQUEST_TIMEOUT,
                    )
        elif program_number is not None:
            session.delete(
                f"{API_URL}/program/{program_number}",
                timeout=REQUEST_TIMEOUT,
            )
