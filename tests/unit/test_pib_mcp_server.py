"""Unit coverage for the schema and safety boundary of the pib MCP server."""

import asyncio

import pytest

from pib_mcp_server.server import ACTUATING_TOOLS, create_server


class FakeBackend:
    def __init__(self):
        self.calls = []
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
        return [{"name": "Home"}]

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


def _call(server, name, arguments):
    result = asyncio.run(server.call_tool(name, arguments))
    return result[1]


def test_tool_discovery_exposes_complete_declared_surface():
    names = {tool.name for tool in _tools(create_server(FakeBackend()))}

    assert names == {
        "pib_list_motors",
        "pib_get_state",
        "pib_list_poses",
        "pib_list_programs",
        "pib_capture_image",
        "pib_move_motor",
        "pib_apply_pose",
        "pib_run_program",
        "pib_set_led",
        "pib_set_relay",
        "pib_soul_append",
    }


def test_every_tool_has_an_object_input_schema():
    for tool in _tools(create_server(FakeBackend())):
        assert tool.inputSchema["type"] == "object"
        assert "properties" in tool.inputSchema


def test_schema_declares_required_parameters_and_constraints():
    tools = {tool.name: tool for tool in _tools(create_server(FakeBackend()))}

    move = tools["pib_move_motor"].inputSchema
    assert set(move["required"]) == {"motor_name", "position"}
    assert move["properties"]["position"]["type"] == "integer"

    led = tools["pib_set_led"].inputSchema["properties"]
    assert led["button_id"]["minimum"] == 1
    assert led["button_id"]["maximum"] == 3
    assert led["red"]["minimum"] == 0
    assert led["red"]["maximum"] == 255

    soul = tools["pib_soul_append"].inputSchema["properties"]["lesson"]
    assert soul["minLength"] == 1
    assert soul["maxLength"] == 500


def test_invalid_schema_rejection_happens_before_backend():
    backend = FakeBackend()
    server = create_server(backend, actuating_enabled=True)

    with pytest.raises(Exception, match="validation error"):
        _call(server, "pib_move_motor", {"motor_name": "head"})

    assert backend.calls == []


@pytest.mark.parametrize("position", [-9001, 9001])
def test_motor_position_range_is_rejected_before_hardware(position):
    backend = FakeBackend()
    result = _call(
        create_server(backend, actuating_enabled=True),
        "pib_move_motor",
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
        "pib_move_motor",
        {"motor_name": "head", "position": position},
    )

    assert result["ok"] is True
    assert ("move_motor", "head", position) in backend.calls


def test_actuating_tools_are_disabled_by_default(monkeypatch):
    monkeypatch.delenv("PIB_MCP_ENABLE_ACTUATION", raising=False)
    backend = FakeBackend()
    server = create_server(backend)

    result = _call(
        server, "pib_move_motor", {"motor_name": "head", "position": 0}
    )

    assert ACTUATING_TOOLS
    assert result["error"]["code"] == "actuation_disabled"
    assert backend.calls == []


def test_soul_append_accepts_exactly_500_characters():
    backend = FakeBackend()
    result = _call(
        create_server(backend),
        "pib_soul_append",
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
            "pib_soul_append",
            {"personality_id": "personality-1", "lesson": "x" * 501},
        )

    assert backend.calls == []
