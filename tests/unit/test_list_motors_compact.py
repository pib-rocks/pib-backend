"""PR-2060/2073: compact MCP payloads, detail=true for full config."""

import asyncio
import json
import time

from pib_mcp_server.backend import PibBackend
from pib_mcp_server.server import create_server
from pib_mcp_payload_assertions import assert_tool_payload_within_budget

COMPACT_MOTOR_KEYS = frozenset({"id", "name", "position", "limits", "unit"})
COMPACT_BYTE_BUDGET = 6000
MOTOR_COUNT = 24

FULL_MOTOR_TEMPLATE = {
    "name": "placeholder_motor",
    "pulseWidthMin": 700,
    "pulseWidthMax": 2500,
    "rotationRangeMin": -4500,
    "rotationRangeMax": 4500,
    "velocity": 500000,
    "acceleration": 10000,
    "deceleration": 5000,
    "period": 19500,
    "turnedOn": True,
    "visible": True,
    "upperEndpoint": "max",
    "controllerNumber": 1,
    "channel": 0,
    "brickletPins": [{"pin": 0, "bricklet": "SHU", "invert": True}],
}


def _motor_names() -> list[str]:
    return [
        "turn_head_motor",
        "tilt_head_motor",
        "right_arm_motor_shoulder",
        "right_arm_motor_upper_arm",
        "right_arm_motor_elbow",
        "right_arm_motor_lower_arm",
        "left_arm_motor_shoulder",
        "left_arm_motor_upper_arm",
        "left_arm_motor_elbow",
        "left_arm_motor_lower_arm",
        "right_leg_motor_hip",
        "right_leg_motor_thigh",
        "right_leg_motor_knee",
        "right_leg_motor_shin",
        "right_leg_motor_ankle",
        "left_leg_motor_hip",
        "left_leg_motor_thigh",
        "left_leg_motor_knee",
        "left_leg_motor_shin",
        "left_leg_motor_ankle",
        "tilt_forward_motor",
        "tilt_backward_motor",
        "right_hand_motor",
        "left_hand_motor",
    ]


def full_motor_fixture() -> list[dict]:
    motors = []
    for index, name in enumerate(_motor_names()):
        motor = dict(FULL_MOTOR_TEMPLATE)
        motor["name"] = name
        motor["channel"] = index % 10
        motor["brickletPins"] = [
            {"pin": index % 10, "bricklet": "SHU", "invert": bool(index % 2)}
        ]
        motors.append(motor)
    assert len(motors) == MOTOR_COUNT
    return motors


def _call(server, name, arguments):
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


class RecordingBackend(PibBackend):
    def __init__(self, motors: list[dict]):
        super().__init__()
        self._motors = motors
        self.ros_calls: list[str] = []

    def get_motors(self):
        return list(self._motors)

    def _http(self, method, path, payload=None):
        if path == "/motor":
            return {"motors": self.get_motors()}
        if path == "/bricklet":
            return {
                "bricklets": [
                    {"uid": "abc", "deviceType": "Servo Bricklet 2.0", "number": 1}
                ]
            }
        if path == "/diagnostics/summary":
            return {"status": "ok", "motorsOnline": MOTOR_COUNT}
        raise AssertionError(f"unexpected HTTP {method} {path}")

    def _ros_service(self, service, service_type, arguments):
        self.ros_calls.append(arguments["joint_name"])
        return {"successful": True, "position": 120 * len(self.ros_calls)}


def test_list_motors_default_is_compact_under_byte_budget():
    backend = RecordingBackend(full_motor_fixture())
    server = create_server(backend)

    response = _call(server, "list_motors", {})
    assert response["ok"] is True
    payload = response["result"]
    assert "bricklets" not in payload
    motors = payload["motors"]
    assert len(motors) == MOTOR_COUNT

    for motor in motors:
        assert set(motor) == COMPACT_MOTOR_KEYS
        assert motor["id"] == motor["name"]
        assert motor["unit"] == "degree"
        assert motor["limits"] == {"min": -45.0, "max": 45.0}
        assert isinstance(motor["position"], float)

    assert_tool_payload_within_budget("list_motors", payload)
    encoded = json.dumps(payload, separators=(",", ":"))
    assert len(encoded.encode("utf-8")) <= COMPACT_BYTE_BUDGET


def test_list_motors_detail_returns_full_motor_dict():
    backend = RecordingBackend(full_motor_fixture())
    server = create_server(backend)

    response = _call(server, "list_motors", {"detail": True})
    assert response["ok"] is True
    payload = response["result"]
    assert "bricklets" in payload
    assert len(payload["bricklets"]) == 1

    for index, motor in enumerate(payload["motors"]):
        source = backend._motors[index]
        for key in source:
            assert key in motor
            assert motor[key] == source[key]
        assert motor["currentPosition"] == (120 * (index + 1)) / 100.0


def test_get_state_default_is_compact_under_budget():
    backend = RecordingBackend(full_motor_fixture())
    server = create_server(backend)

    response = _call(server, "get_state", {})
    assert response["ok"] is True
    payload = response["result"]
    assert payload["diagnostics"] == {"status": "ok", "motorsOnline": MOTOR_COUNT}
    assert set(payload["motors"][0]) == COMPACT_MOTOR_KEYS
    assert "bricklets" not in payload
    assert_tool_payload_within_budget("get_state", payload)


def test_get_state_detail_returns_full_motors_and_diagnostics():
    backend = RecordingBackend(full_motor_fixture())
    server = create_server(backend)

    response = _call(server, "get_state", {"detail": True})
    assert response["ok"] is True
    payload = response["result"]
    assert payload["diagnostics"] == {"status": "ok", "motorsOnline": MOTOR_COUNT}
    assert "bricklets" in payload
    assert "velocity" in payload["motors"][0]
    assert "currentPosition" in payload["motors"][0]


def test_list_motors_tool_call_latency_under_two_seconds():
    backend = RecordingBackend(full_motor_fixture())
    server = create_server(backend)
    started = time.monotonic()
    response = _call(server, "list_motors", {})
    elapsed = time.monotonic() - started
    assert response["ok"] is True
    assert elapsed <= 2.0


def test_pib_backend_list_motors_detail_false_omits_bricklets():
    backend = RecordingBackend(full_motor_fixture())
    payload = backend.list_motors(detail=False)
    assert "bricklets" not in payload
    assert set(payload["motors"][0]) == COMPACT_MOTOR_KEYS
