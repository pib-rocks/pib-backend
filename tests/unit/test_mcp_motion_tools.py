"""MCP multi-motor motion tools and ROS trajectory wire shape (PR-2077)."""

from __future__ import annotations

import asyncio
import json
import math
from typing import Any

import pytest

from pib_mcp_server.backend import BackendError, PibBackend
from pib_mcp_server.server import create_server


def _call(server, name: str, arguments: dict) -> dict:
    result = asyncio.run(server.call_tool(name, arguments))
    if isinstance(result, tuple):
        return result[1]
    if hasattr(result, "structured_output") and result.structured_output is not None:
        return result.structured_output
    if hasattr(result, "content") and result.content:
        c0 = result.content[0]
        if hasattr(c0, "text"):
            return json.loads(c0.text)
        return c0
    return result


class TrajectoryRecordingBackend(PibBackend):
    def __init__(self, motors: list[dict[str, Any]]):
        super().__init__()
        self._motors = motors
        self.ros_calls: list[tuple[str, str, dict[str, Any]]] = []
        self._next_response: dict[str, Any] = {"successful": True, "results": []}

    def get_motors(self):
        return list(self._motors)

    def _ros_service(
        self, service: str, service_type: str, arguments: dict[str, Any]
    ) -> dict[str, Any]:
        self.ros_calls.append((service, service_type, arguments))
        return dict(self._next_response)

    def set_ros_response(self, response: dict[str, Any]) -> None:
        self._next_response = response


MOTORS = [
    {
        "name": "head",
        "rotationRangeMin": -9000,
        "rotationRangeMax": 9000,
    },
    {
        "name": "arm",
        "rotationRangeMin": -4500,
        "rotationRangeMax": 4500,
    },
]


def test_move_motors_sends_one_point_with_positions_in_joint_order():
    backend = TrajectoryRecordingBackend(MOTORS)
    server = create_server(backend, actuating_enabled=True)

    response = _call(
        server,
        "move_motors",
        {
            "moves": [
                {"motor": "head", "position_degrees": 90.0},
                {"motor": "arm", "position_degrees": 45.0},
            ]
        },
    )
    assert response["ok"] is True
    assert len(backend.ros_calls) == 1
    trajectory = backend.ros_calls[0][2]["joint_trajectory"]
    assert trajectory["joint_names"] == ["head", "arm"]
    assert len(trajectory["points"]) == 1
    positions = trajectory["points"][0]["positions"]
    assert positions == [math.pi / 2, math.radians(45.0)]


def test_move_motors_degree_to_radian_conversion_is_exact_for_ninety_degrees():
    backend = TrajectoryRecordingBackend([MOTORS[0]])
    server = create_server(backend, actuating_enabled=True)

    _call(
        server,
        "move_motors",
        {"moves": [{"motor": "head", "position_degrees": 90}]},
    )
    positions = backend.ros_calls[0][2]["joint_trajectory"]["points"][0]["positions"]
    assert positions[0] == pytest.approx(math.pi / 2)


def test_invalid_move_rejects_whole_call_without_ros_service():
    backend = TrajectoryRecordingBackend(MOTORS)
    server = create_server(backend, actuating_enabled=True)

    response = _call(
        server,
        "move_motors",
        {
            "moves": [
                {"motor": "head", "position_degrees": 0},
                {"motor": "arm", "position_degrees": 91.0},
            ]
        },
    )
    assert response["ok"] is False
    assert response["error"]["code"] == "position_out_of_range"
    assert backend.ros_calls == []


def test_per_motor_results_and_error_code_propagate():
    backend = TrajectoryRecordingBackend([MOTORS[0]])
    backend.set_ros_response(
        {
            "successful": False,
            "results": [
                {
                    "motor_name": "head",
                    "successful": False,
                    "error_code": "out_of_range",
                    "detail": "target rejected",
                }
            ],
        }
    )
    server = create_server(backend, actuating_enabled=True)

    response = _call(
        server,
        "move_motors",
        {"moves": [{"motor": "head", "position_degrees": 10.0}]},
    )
    assert response["ok"] is False
    assert response["error"]["code"] == "motion_failed"
    failed = response["error"]["details"]["results"]
    assert failed[0]["motorName"] == "head"
    assert failed[0]["errorCode"] == "out_of_range"
    assert failed[0]["detail"] == "target rejected"


def test_apply_pose_builds_standard_joint_trajectory_shape():
    backend = TrajectoryRecordingBackend(MOTORS)
    pose_payload = {
        "motorPositions": [
            {"motorName": "head", "position": 9000},
            {"motorName": "arm", "position": 4500},
        ]
    }

    def fake_http(method, path, payload=None):
        assert method == "GET"
        assert path == "/pose/by-name/Wave"
        return pose_payload

    backend._http = fake_http
    server = create_server(backend, actuating_enabled=True)

    response = _call(server, "apply_pose", {"pose_name": "Wave"})
    assert response["ok"] is True
    trajectory = backend.ros_calls[0][2]["joint_trajectory"]
    assert trajectory["joint_names"] == ["head", "arm"]
    assert len(trajectory["points"]) == 1
    assert trajectory["points"][0]["positions"] == [
        math.pi / 2,
        math.radians(45.0),
    ]


def test_pib_backend_move_motor_delegates_to_move_motors():
    backend = TrajectoryRecordingBackend([MOTORS[0]])
    backend.set_ros_response(
        {
            "successful": True,
            "results": [
                {
                    "motor_name": "head",
                    "successful": True,
                    "error_code": "",
                    "detail": "",
                }
            ],
        }
    )
    result = backend.move_motor("head", 15.0)
    assert result["motorName"] == "head"
    assert result["positionDegrees"] == 15.0
    assert result["successful"] is True
    trajectory = backend.ros_calls[0][2]["joint_trajectory"]
    assert len(trajectory["points"]) == 1
    assert len(trajectory["points"][0]["positions"]) == 1


def test_apply_joint_trajectory_includes_time_from_start():
    backend = TrajectoryRecordingBackend([MOTORS[0]])
    server = create_server(backend, actuating_enabled=True)

    response = _call(
        server,
        "apply_joint_trajectory",
        {
            "joint_names": ["head"],
            "points": [
                {"positions_degrees": [0.0], "time_from_start_sec": 0.0},
                {"positions_degrees": [30.0], "time_from_start_sec": 1.5},
            ],
        },
    )
    assert response["ok"] is True
    points = backend.ros_calls[0][2]["joint_trajectory"]["points"]
    assert len(points) == 2
    assert points[1]["time_from_start"] == {"sec": 1, "nanosec": 500000000}


def test_apply_joint_trajectory_backend_raises_with_result_details():
    backend = TrajectoryRecordingBackend([MOTORS[0]])
    backend.set_ros_response(
        {
            "successful": False,
            "results": [
                {
                    "motor_name": "head",
                    "successful": False,
                    "error_code": "legacy_shape_rejected",
                    "detail": "use standard layout",
                }
            ],
        }
    )
    with pytest.raises(BackendError) as exc:
        backend.apply_joint_trajectory(
            ["head"], [{"positions_degrees": [5.0], "time_from_start_sec": 0.0}]
        )
    assert exc.value.code == "trajectory_failed"
    assert exc.value.details["results"][0]["errorCode"] == "legacy_shape_rejected"
