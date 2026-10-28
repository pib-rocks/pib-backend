"""PR-2073: every MCP read tool stays within its declared compact payload budget."""

from __future__ import annotations

import asyncio
import json
from typing import Any
from urllib.parse import unquote

import pytest

from pib_mcp_server.backend import PibBackend
from pib_mcp_server.server import TOOL_PAYLOAD_BUDGETS, create_server
from pib_mcp_server.tool_budgets import (
    payload_utf8_byte_length,
    tool_payload_budget_bytes,
)
from pib_mcp_payload_assertions import assert_tool_payload_within_budget
from test_list_motors_compact import MOTOR_COUNT, full_motor_fixture

PROGRAM_COUNT = 20


def _call(server, name: str, arguments: dict) -> dict:
    result = asyncio.run(server.call_tool(name, arguments))
    if isinstance(result, tuple):
        return result[1]
    if hasattr(result, "structured_output") and result.structured_output is not None:
        return result.structured_output
    content = getattr(result, "content", None)
    if content:
        return json.loads(content[0].text)
    return result


def program_fixture() -> list[dict[str, Any]]:
    programs = []
    for index in range(PROGRAM_COUNT):
        programs.append(
            {
                "name": f"Program {index:02d}",
                "programNumber": f"prog-{index:04d}",
                "codeVisual": json.dumps({"blocks": ["x" * 200] * 10}),
            }
        )
    return programs


def pose_fixture() -> list[dict[str, Any]]:
    return [
        {
            "poseId": index,
            "name": f"Pose {index:02d}",
            "deletable": index > 1,
            "motorPositions": [
                {"motorName": name, "position": index * 10}
                for name in [item["name"] for item in full_motor_fixture()[:6]]
            ],
        }
        for index in range(12)
    ]


class BudgetBackend(PibBackend):
    def __init__(self):
        super().__init__()
        self._motors = full_motor_fixture()
        self._programs = program_fixture()
        self._poses = pose_fixture()

    def get_motors(self):
        return list(self._motors)

    def _ros_service(self, service, service_type, arguments):
        if service == "/get_camera_image":
            return {"imageBase64": "A" * 50_000}
        if service == "/get_joint_position":
            return {"successful": True, "position": 0}
        raise AssertionError(service)

    def _http(self, method, path, payload=None):
        if path == "/motor":
            return {"motors": self.get_motors()}
        if path == "/bricklet":
            return {"bricklets": [{"uid": "abc", "deviceType": "Servo", "number": 1}]}
        if path == "/diagnostics/summary":
            return {
                "overallStatus": "ok",
                "hardwareVariant": "pib4edu",
                "cpuStatus": "ok",
                "memoryStatus": "ok",
                "diskStatus": "ok",
                "containersStatus": "ok",
                "brickletsStatus": "ok",
                "cpuTemperature": 42.0,
                "memoryUsage": {"percentUsed": 12.0, "usedMb": 100, "totalMb": 8000},
                "diskSpace": {"percentUsed": 30.0, "usedGb": 10, "totalGb": 64},
                "healthyContainersCount": 6,
                "totalContainersCount": 6,
                "totalBrickletsCount": 3,
                "totalControllersCount": 3,
                "controllerCounts": {"SHU": 2, "SMU": 1},
            }
        if path == "/program":
            return {
                "programs": [
                    {"name": p["name"], "programNumber": p["programNumber"]}
                    for p in self._programs
                ]
            }
        if path.startswith("/program/") and path.endswith("/code"):
            number = path.split("/")[2]
            match = next(p for p in self._programs if p["programNumber"] == number)
            return {"codeVisual": match["codeVisual"]}
        if path == "/pose":
            return {
                "poses": [
                    {
                        "poseId": pose["poseId"],
                        "name": pose["name"],
                        "deletable": pose["deletable"],
                    }
                    for pose in self._poses
                ]
            }
        if path.startswith("/pose/by-name/"):
            name = unquote(path.removeprefix("/pose/by-name/"))
            return next(p for p in self._poses if p["name"] == name)
        if path == "/system/motion-model":
            return {
                "schemaVersion": 1,
                "units": {"angle": "centidegree"},
                "chains": {"head": {"joints": [{"motor": "turn_head_motor"}] * 14}},
                "postures": {f"Pose {i:02d}": {} for i in range(10)},
                "prompt": "Body (centidegrees: ...)" + (" x" * 400),
            }
        if path.startswith("/smart-home/devices"):
            return {
                "devices": [
                    {
                        "entityId": f"light.room_{index}",
                        "friendlyName": f"Room {index} Light",
                        "room": f"Room {index}",
                        "domain": "light",
                        "state": "on",
                        "exposeToVoice": True,
                        "attributes": {"brightness": 200, "color": "#ffffff"},
                    }
                    for index in range(8)
                ]
            }
        raise AssertionError(f"unexpected HTTP {method} {path}")

    def move_motor(self, motor_name: str, position: int) -> dict[str, Any]:
        return {"motorName": motor_name, "position": position}

    def apply_pose(self, pose_name: str) -> dict[str, Any]:
        return {"poseName": pose_name, "motorCount": 6}

    def run_program(self, program_id: str) -> dict[str, Any]:
        return {"programId": program_id, "goalId": "goal-1"}

    def set_led(
        self, button_id: int, red: int, green: int, blue: int
    ) -> dict[str, Any]:
        return {
            "buttonId": button_id,
            "color": {"red": red, "green": green, "blue": blue},
        }

    def set_relay(self, turned_on: bool) -> dict[str, Any]:
        return {"turnedOn": turned_on}

    def soul_append(self, personality_id: str, lesson: str, detail: bool = False):
        result = {"personalityId": personality_id, "appended": True}
        if detail:
            result["personality"] = {"id": personality_id, "lesson": lesson}
        return result


@pytest.mark.parametrize(
    ("tool_name", "arguments"),
    [
        ("list_motors", {}),
        ("get_state", {}),
        ("list_programs", {}),
        ("list_poses", {}),
        ("capture_image", {}),
        ("get_motion_model", {}),
        ("list_smarthome_devices", {}),
        ("get_smarthome_state", {"device": "Room 0 Light"}),
        ("move_motor", {"motor_name": "turn_head_motor", "position_degrees": 0}),
        ("apply_pose", {"pose_name": "Pose 02"}),
        ("run_program", {"program_id": "prog-0001"}),
        ("set_led", {"button_id": 1, "red": 0, "green": 0, "blue": 0}),
        ("set_relay", {"turned_on": True}),
        ("soul_append", {"personality_id": "p1", "lesson": "hi"}),
    ],
)
def test_default_tool_payloads_stay_within_budget(tool_name, arguments):
    backend = BudgetBackend()
    server = create_server(backend, actuating_enabled=True)
    response = _call(server, tool_name, arguments)
    assert response["ok"] is True
    assert_tool_payload_within_budget(tool_name, response["result"])


@pytest.mark.parametrize(
    ("tool_name", "arguments"),
    [
        ("list_motors", {"detail": True}),
        ("get_state", {"detail": True}),
        ("list_programs", {"detail": True}),
        ("list_poses", {"detail": True}),
        ("capture_image", {"detail": True}),
        ("get_motion_model", {"detail": True}),
    ],
)
def test_detail_mode_returns_richer_payload_than_compact(tool_name, arguments):
    backend = BudgetBackend()
    server = create_server(backend, actuating_enabled=True)
    compact = _call(server, tool_name, {})["result"]
    detailed = _call(server, tool_name, arguments)["result"]
    assert payload_utf8_byte_length(detailed) >= payload_utf8_byte_length(compact)
    if tool_name == "capture_image":
        assert "imageBase64" in detailed
        assert "imageBase64" not in compact
    if tool_name == "list_motors":
        assert "bricklets" in detailed
        assert "bricklets" not in compact


def test_assert_budget_helper_reports_tool_name_and_bytes():
    with pytest.raises(
        AssertionError, match=r"list_motors payload \d+ bytes exceeds budget"
    ):
        assert_tool_payload_within_budget("list_motors", {"motors": ["x" * 10_000]})


def test_server_declares_budget_for_every_discovered_tool():
    backend = BudgetBackend()
    server = create_server(backend)
    names = {tool.name for tool in asyncio.run(server.list_tools())}
    for name in names:
        expected = TOOL_PAYLOAD_BUDGETS.get(name, TOOL_PAYLOAD_BUDGETS["default"])
        assert tool_payload_budget_bytes(name) == expected


def test_list_programs_fixture_count():
    assert len(program_fixture()) == PROGRAM_COUNT


def test_list_motors_fixture_count():
    assert len(full_motor_fixture()) == MOTOR_COUNT
