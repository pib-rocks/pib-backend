"""Unit coverage for the schema and safety boundary of the pib MCP server."""

import asyncio

import pytest

from pib_mcp_server.backend import BackendError, PibBackend
from pib_mcp_server.server import ACTUATING_TOOLS, create_server


class FakeBackend:
    def __init__(self):
        self.calls = []
        self.poses = [
            {"name": "Startup/Resting", "deletable": False},
            {"name": "Calibration", "deletable": False},
        ]
        self.motors = [
            {
                "name": "head",
                "rotationRangeMin": -9000,
                "rotationRangeMax": 9000,
            }
        ]

    def get_motors(self):
        self.calls.append(("get_motors",))
        return self.motors

    def list_motors(self, detail=False):
        if detail:
            return {"motors": self.motors, "bricklets": []}
        return {
            "motors": [
                {
                    "name": motor["name"],
                    "position": motor.get("currentPosition"),
                    "limits": {
                        "min": motor.get("rotationRangeMin"),
                        "max": motor.get("rotationRangeMax"),
                    },
                    "unit": "degree",
                }
                for motor in self.motors
            ]
        }

    def get_state(self, detail=False):
        if detail:
            state = self.list_motors(detail=True)
            state["diagnostics"] = {}
            return state
        return {"motors": self.list_motors()["motors"], "diagnostics": {}}

    def list_poses(self, detail=False):
        if detail:
            return self.poses
        return [
            {
                "poseId": pose.get("poseId", pose.get("pose_id")),
                "name": pose["name"],
                "deletable": pose.get("deletable"),
            }
            for pose in self.poses
        ]

    def list_programs(self, detail=False):
        programs = [{"programNumber": "7", "name": "Wave"}]
        if detail:
            return [{**programs[0], "codeVisual": "{}"}]
        return programs

    def capture_image(self, detail=False):
        if detail:
            return {"imageBase64": "aGVsbG8=", "mimeType": "image/jpeg"}
        return {
            "imageRef": "abcd",
            "mimeType": "image/jpeg",
            "byteLength": 5,
            "sha256": "0" * 64,
        }

    def move_motors(self, moves):
        self.calls.append(("move_motors", moves))
        return {
            "results": [
                {
                    "motorName": move["motor"],
                    "positionDegrees": move["position_degrees"],
                    "successful": True,
                    "errorCode": "",
                    "detail": "",
                }
                for move in moves
            ]
        }

    def move_motor(self, motor_name, position_degrees):
        self.calls.append(("move_motor", motor_name, position_degrees))
        return {
            "motorName": motor_name,
            "positionDegrees": position_degrees,
            "successful": True,
            "errorCode": "",
            "detail": "",
        }

    def apply_pose(self, pose_name):
        self.calls.append(("apply_pose", pose_name))
        return {"poseName": pose_name}

    def run_program(self, program_id):
        self.calls.append(("run_program", program_id))
        return {"programId": program_id, "goalId": "goal-1"}

    def set_led(self, button_id, red, green, blue):
        self.calls.append(("set_led", button_id, red, green, blue))
        return {"buttonId": button_id}

    def set_relay(self, turned_on):
        self.calls.append(("set_relay", turned_on))
        return {"turnedOn": turned_on}

    def soul_append(self, personality_id, lesson, detail=False):
        self.calls.append(("soul_append", personality_id, lesson, detail))
        result = {"personalityId": personality_id, "appended": True}
        if detail:
            result["personality"] = {"id": personality_id}
        return result

    def list_smarthome_devices(self, room=None, domain=None, detail=False):
        self.calls.append(("list_smarthome_devices", room, domain, detail))
        devices = [
            {
                "entityId": "light.living_room",
                "friendlyName": "Living Room",
                "room": "Living Room",
                "domain": "light",
                "state": "on",
                "exposeToVoice": True,
            },
            {
                "entityId": "switch.hidden",
                "friendlyName": "Hidden Switch",
                "room": "Utility",
                "domain": "switch",
                "state": "off",
                "exposeToVoice": False,
            },
        ]
        filtered = devices
        if room:
            filtered = [d for d in filtered if d.get("room") == room]
        if domain:
            filtered = [d for d in filtered if d.get("domain") == domain]
        devices_out = (
            filtered
            if detail
            else [
                {
                    key: device[key]
                    for key in (
                        "entityId",
                        "friendlyName",
                        "room",
                        "domain",
                        "state",
                        "exposeToVoice",
                    )
                    if key in device
                }
                for device in filtered
            ]
        )
        return {"devices": devices_out}

    def get_smarthome_state(self, target, detail=False):
        self.calls.append(("get_smarthome_state", target, detail))
        for device in self.list_smarthome_devices()["devices"]:
            if device["entityId"] == target or device["friendlyName"] == target:
                return device
        raise Exception("device_not_found")

    def control_smarthome_device(self, target, action, value=None):
        self.calls.append(("control_smarthome_device", target, action, value))
        if target == "Hidden Switch":
            raise Exception("voice_access_denied")
        return {
            "entityId": "light.living_room",
            "state": "off" if action == "turn_off" else "on",
            "action": action,
        }

    def trigger_smarthome_scene(self, scene_name):
        self.calls.append(("trigger_smarthome_scene", scene_name))
        return self.control_smarthome_device(scene_name, "activate")


def _tools(server):
    return asyncio.run(server.list_tools())


def _get_schema(tool):
    schema = getattr(tool, "input_schema", None)
    if schema is None:
        schema = getattr(tool, "inputSchema", None)
    if isinstance(schema, dict):
        return schema
    if hasattr(schema, "model_dump"):
        return schema.model_dump()
    return tool.model_dump().get("inputSchema") or tool.model_dump().get("input_schema")


def _call(server, name, arguments):
    result = asyncio.run(server.call_tool(name, arguments))
    if isinstance(result, tuple):
        return result[1]
    if hasattr(result, "structured_output") and result.structured_output is not None:
        return result.structured_output
    if hasattr(result, "content") and result.content:
        # Check if content has text/json
        c0 = result.content[0]
        if hasattr(c0, "text"):
            import json

            try:
                return json.loads(c0.text)
            except Exception:
                return c0.text
        return c0
    return result


def test_list_poses_returns_default_poses():
    backend = FakeBackend()
    backend.poses = [
        {"poseId": 1, "name": "Startup/Resting", "deletable": False},
        {"poseId": 2, "name": "Calibration", "deletable": False},
    ]
    server = create_server(backend)
    res = _call(server, "list_poses", {})
    assert res["ok"] is True
    pose_names = [p["name"] for p in res["result"]]
    assert "Startup/Resting" in pose_names
    assert "Calibration" in pose_names


def test_tool_discovery_exposes_complete_declared_surface():
    names = {tool.name for tool in _tools(create_server(FakeBackend()))}

    assert names == {
        "list_motors",
        "get_motion_model",
        "get_state",
        "list_poses",
        "list_programs",
        "capture_image",
        "move_motor",
        "move_motors",
        "apply_joint_trajectory",
        "apply_pose",
        "run_program",
        "set_led",
        "set_relay",
        "list_smarthome_devices",
        "get_smarthome_state",
        "control_smarthome_device",
        "trigger_smarthome_scene",
        "soul_append",
    }


def test_every_tool_has_an_object_input_schema():
    for tool in _tools(create_server(FakeBackend())):
        schema = _get_schema(tool)
        assert schema["type"] == "object"
        assert "properties" in schema


def test_schema_declares_required_parameters_and_constraints():
    tools = {tool.name: tool for tool in _tools(create_server(FakeBackend()))}
    move = _get_schema(tools["move_motor"])
    assert "motor_name" in move["properties"]
    assert "position_degrees" in move["properties"]
    assert move["properties"]["position_degrees"]["type"] == "number"

    led = _get_schema(tools["set_led"])["properties"]
    assert led["button_id"]["minimum"] == 1
    assert led["button_id"]["maximum"] == 3
    assert led["red"]["minimum"] == 0
    assert led["red"]["maximum"] == 255

    soul = _get_schema(tools["soul_append"])["properties"]["lesson"]
    assert soul["minLength"] == 1
    assert soul["maxLength"] == 500


def test_invalid_schema_rejection_happens_before_backend():
    backend = FakeBackend()
    server = create_server(backend, actuating_enabled=True)

    with pytest.raises(Exception, match="validation error"):
        _call(server, "move_motor", {"motor_name": "head"})

    assert backend.calls == []


@pytest.mark.parametrize("position_degrees", [-90.1, 90.1])
def test_motor_position_range_is_rejected_before_hardware(position_degrees):
    backend = FakeBackend()
    result = _call(
        create_server(backend, actuating_enabled=True),
        "move_motor",
        {"motor_name": "head", "position_degrees": position_degrees},
    )

    assert result["ok"] is False
    assert result["error"]["code"] == "position_out_of_range"
    assert not any(call[0] == "move_motor" for call in backend.calls)


@pytest.mark.parametrize("position_degrees", [-90.0, 90.0])
def test_motor_position_at_configured_boundary_is_allowed(position_degrees):
    backend = FakeBackend()
    result = _call(
        create_server(backend, actuating_enabled=True),
        "move_motor",
        {"motor_name": "head", "position_degrees": position_degrees},
    )

    assert result["ok"] is True
    assert ("move_motor", "head", position_degrees) in backend.calls


def test_actuating_tools_are_disabled_by_default(monkeypatch):
    monkeypatch.delenv("PIB_MCP_ENABLE_ACTUATION", raising=False)
    backend = FakeBackend()
    server = create_server(backend)

    result = _call(server, "move_motor", {"motor_name": "head", "position_degrees": 0})

    assert ACTUATING_TOOLS
    assert result["error"]["code"] == "actuation_disabled"
    assert backend.calls == []


def test_soul_append_accepts_exactly_500_characters():
    backend = FakeBackend()
    result = _call(
        create_server(backend),
        "soul_append",
        {"personality_id": "personality-1", "lesson": "x" * 500},
    )

    assert result["ok"] is True
    assert backend.calls == [("soul_append", "personality-1", "x" * 500, False)]


def test_list_smarthome_devices_returns_voice_devices():
    backend = FakeBackend()
    result = _call(create_server(backend), "list_smarthome_devices", {})
    assert result["ok"] is True
    assert result["result"]["devices"][0]["entityId"] == "light.living_room"
    assert ("list_smarthome_devices", None, None, False) in backend.calls


def test_get_smarthome_state_resolves_device():
    backend = FakeBackend()
    result = _call(
        create_server(backend),
        "get_smarthome_state",
        {"device": "Living Room"},
    )
    assert result["ok"] is True
    assert result["result"]["friendlyName"] == "Living Room"


def test_control_smarthome_device_parses_numeric_value_and_requires_actuation():
    backend = FakeBackend()
    disabled = _call(
        create_server(backend),
        "control_smarthome_device",
        {"device": "Living Room", "action": "set_brightness", "value": "50"},
    )
    assert disabled["error"]["code"] == "actuation_disabled"

    result = _call(
        create_server(backend, actuating_enabled=True),
        "control_smarthome_device",
        {"device": "Living Room", "action": "set_brightness", "value": "50"},
    )
    assert result["ok"] is True
    assert (
        "control_smarthome_device",
        "Living Room",
        "set_brightness",
        50,
    ) in backend.calls


def test_pib_backend_get_smarthome_state_resolves_friendly_name():
    backend = PibBackend()
    devices = [
        {
            "entityId": "light.kitchen",
            "friendlyName": "Kitchen Light",
            "room": "Kitchen",
            "exposeToVoice": True,
            "state": "on",
        }
    ]
    backend._http = lambda method, path, payload=None: {"devices": devices}
    device = backend.get_smarthome_state("kitchen light")
    assert device["entityId"] == "light.kitchen"


def test_pib_backend_get_smarthome_state_raises_when_missing():
    backend = PibBackend()
    backend._http = lambda method, path, payload=None: {"devices": []}
    with pytest.raises(BackendError) as exc:
        backend.get_smarthome_state("missing")
    assert exc.value.code == "device_not_found"


def test_pib_backend_control_smarthome_device_denies_non_voice_device():
    backend = PibBackend()
    backend._http = lambda method, path, payload=None: {
        "devices": [
            {
                "entityId": "switch.secret",
                "friendlyName": "Secret",
                "exposeToVoice": False,
            }
        ]
    }
    with pytest.raises(BackendError) as exc:
        backend.control_smarthome_device("Secret", "turn_on")
    assert exc.value.code == "voice_access_denied"


def test_pib_backend_control_smarthome_device_posts_resolved_entity():
    backend = PibBackend()
    calls = []

    def fake_http(method, path, payload=None):
        calls.append((method, path, payload))
        if method == "GET":
            return {
                "devices": [
                    {
                        "entityId": "light.living_room",
                        "friendlyName": "Living Room",
                        "exposeToVoice": True,
                    }
                ]
            }
        return {"entityId": "light.living_room", "state": "on", "applied": True}

    backend._http = fake_http
    result = backend.control_smarthome_device("Living Room", "turn_on")
    assert result["state"] == "on"
    assert calls[-1][0] == "POST"
    assert calls[-1][1] == "/smart-home/devices/light.living_room/action"
    assert calls[-1][2] == {"action": "turn_on"}


def test_trigger_smarthome_scene_activates_scene_when_enabled():
    backend = FakeBackend()
    result = _call(
        create_server(backend, actuating_enabled=True),
        "trigger_smarthome_scene",
        {"scene": "Movie Night"},
    )
    assert result["ok"] is True
    assert ("trigger_smarthome_scene", "Movie Night") in backend.calls


def test_soul_append_rejects_oversized_lesson_before_backend():
    backend = FakeBackend()
    server = create_server(backend)

    with pytest.raises(Exception, match="500 characters"):
        _call(
            server,
            "soul_append",
            {"personality_id": "personality-1", "lesson": "x" * 501},
        )

    assert backend.calls == []
