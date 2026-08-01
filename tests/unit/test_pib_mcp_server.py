"""Unit coverage for the schema and safety boundary of the pib MCP server."""

import asyncio

import pytest

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

    def list_motors(self):
        return {"motors": self.motors, "bricklets": []}

    def get_state(self):
        return {"motors": self.motors, "diagnostics": {}}

    def list_poses(self):
        return self.poses

    def list_programs(self):
        return [{"programNumber": "7", "name": "Wave"}]

    def capture_image(self):
        return {"imageBase64": "aGVsbG8=", "mimeType": "image/jpeg"}

    def move_motor(self, motor_name, position):
        self.calls.append(("move_motor", motor_name, position))
        return {"motorName": motor_name, "position": position}

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

    def soul_append(self, personality_id, lesson):
        self.calls.append(("soul_append", personality_id, lesson))
        return {"appended": True}


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
        {"name": "Startup/Resting", "deletable": False},
        {"name": "Calibration", "deletable": False},
    ]
    backend.list_poses = lambda: backend.poses
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
        "get_state",
        "list_poses",
        "list_programs",
        "capture_image",
        "move_motor",
        "apply_pose",
        "run_program",
        "set_led",
        "set_relay",
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
    assert "position" in move["properties"]
    assert move["properties"]["position"]["type"] == "integer"

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


@pytest.mark.parametrize("position", [-9001, 9001])
def test_motor_position_range_is_rejected_before_hardware(position):
    backend = FakeBackend()
    result = _call(
        create_server(backend, actuating_enabled=True),
        "move_motor",
        {"motor_name": "head", "position": position},
    )

    assert result["ok"] is False
    assert result["error"]["code"] == "position_out_of_range"
    assert not any(call[0] == "move_motor" for call in backend.calls)


@pytest.mark.parametrize("position", [-9000, 9000])
def test_motor_position_at_configured_boundary_is_allowed(position):
    backend = FakeBackend()
    result = _call(
        create_server(backend, actuating_enabled=True),
        "move_motor",
        {"motor_name": "head", "position": position},
    )

    assert result["ok"] is True
    assert ("move_motor", "head", position) in backend.calls


def test_actuating_tools_are_disabled_by_default(monkeypatch):
    monkeypatch.delenv("PIB_MCP_ENABLE_ACTUATION", raising=False)
    backend = FakeBackend()
    server = create_server(backend)

    result = _call(
        server, "move_motor", {"motor_name": "head", "position": 0}
    )

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
    assert backend.calls == [("soul_append", "personality-1", "x" * 500)]


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
