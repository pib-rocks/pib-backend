"""Live interaction quality: VAD ownership, self-hearing, robot actions."""

import asyncio
import inspect
from pathlib import Path

from pib_hermes_config.live_interaction import (
    FACE_LISTENING_TEXT,
    INTERRUPTION_REPORT,
    PLAYBACK_TAIL_EXCLUSION_SECONDS,
    admit_live_uplink,
    consume_playback,
    face_for_hardware_vad,
    gemini_function_declarations,
    interruption_detected,
    iter_function_calls,
    live_turn_detection_config,
    on_interruption,
    perform_robot_action,
    personality_requests_actuation,
    turn_based_is_speech,
)
from pib_mcp_server.server import create_server


def _call(server, name, arguments):
    result = asyncio.run(server.call_tool(name, arguments))
    if isinstance(result, tuple):
        return result[1]
    structured = getattr(result, "structured_output", None)
    if structured is not None:
        return structured
    content = getattr(result, "content", None)
    if content:
        text = getattr(content[0], "text", None)
        if isinstance(text, str):
            import json

            return json.loads(text)
    return result


REPO_ROOT = Path(__file__).resolve().parents[2]
AUDIO_LOOP = (
    REPO_ROOT / "ros_packages" / "voice_assistant" / "voice_assistant" / "audio_loop.py"
)
AUDIO_RECORDER = (
    REPO_ROOT
    / "ros_packages"
    / "voice_assistant"
    / "voice_assistant"
    / "audio_recorder.py"
)
EXPRESSION_MANAGER = (
    REPO_ROOT / "ros_packages" / "display" / "display" / "expression_manager.py"
)
DOA_PUBLISHER = (
    REPO_ROOT / "ros_packages" / "ros_audio_io" / "ros_audio_io" / "doa_publisher.py"
)


class _Pause:
    def __init__(self):
        self.events = []
        self.paused = False

    def begin(self):
        self.paused = True
        self.events.append("pause")

    def end(self):
        self.paused = False
        self.events.append("resume")


def test_live_uplink_is_not_gated_by_hardware_vad():
    assert "voice_activity" not in inspect.signature(admit_live_uplink).parameters
    assert admit_live_uplink(now=10.0, exclude_until=0.0, session_paused=False) is True
    assert (
        admit_live_uplink(now=10.0, exclude_until=10.2, session_paused=False) is False
    )
    assert admit_live_uplink(now=10.0, exclude_until=0.0, session_paused=True) is False

    detection = live_turn_detection_config()["realtime_input_config"]
    assert detection["automatic_activity_detection"]["disabled"] is False
    assert detection["activity_handling"] == "START_OF_ACTIVITY_INTERRUPTS"

    loop = AUDIO_LOOP.read_text(encoding="utf-8")
    assert "admit_live_uplink(" in loop
    assert "live_turn_detection_config(" in loop
    assert "voice_activity" not in loop
    assert "activity_start" not in loop


def test_hardware_vad_drives_the_turn_based_path_the_face_and_the_led_ring():
    assert turn_based_is_speech(True, amplitude_silent=True) is True
    assert turn_based_is_speech(False, amplitude_silent=False) is False
    assert turn_based_is_speech(None, amplitude_silent=False) is True
    assert turn_based_is_speech(None, amplitude_silent=True) is False
    assert face_for_hardware_vad(True) == FACE_LISTENING_TEXT
    assert face_for_hardware_vad(False) is None

    recorder = AUDIO_RECORDER.read_text(encoding="utf-8")
    assert "turn_based_is_speech(" in recorder
    assert '"/voice_activity"' in recorder

    face = EXPRESSION_MANAGER.read_text(encoding="utf-8")
    assert '"/voice_activity"' in face
    assert FACE_LISTENING_TEXT in face
    assert "show_default_animation" in face

    ring = DOA_PUBLISHER.read_text(encoding="utf-8")
    assert 'self.tuning.read("VOICEACTIVITY")' in ring
    assert 'Bool, "/voice_activity"' in ring
    assert "set_vad_led" in ring


def test_interruption_stops_playback_drops_the_tail_and_reports_the_cut():
    decision = on_interruption(5.0)
    assert decision["stop_playback"] is True
    assert decision["drop_queue"] is True
    assert decision["exclude_until"] == 5.0 + PLAYBACK_TAIL_EXCLUSION_SECONDS
    assert decision["report"] == INTERRUPTION_REPORT
    assert "cut off" in decision["report"]

    class _Content:
        interrupted = True

    class _Quiet:
        interrupted = False

    assert interruption_detected(_Content()) is True
    assert interruption_detected(_Quiet()) is False

    outcome = consume_playback(b"abcdefghij", slice_bytes=4, cancel_after_slices=1)
    assert outcome["written"] == b"abcd"
    assert outcome["dropped"] == b"efghij"
    assert outcome["stopped_early"] is True

    loop = AUDIO_LOOP.read_text(encoding="utf-8")
    assert "on_interruption(" in loop
    assert 'send_realtime_input(text=decision["report"])' in loop


def test_robot_action_is_announced_before_it_runs_and_the_session_pauses():
    pause = _Pause()
    events = []

    def speak(text):
        assert pause.paused is True
        events.append(("speak", text))

    def execute(name, arguments):
        assert pause.paused is True
        events.append(("run", name, arguments))
        return {"ok": True}

    result = perform_robot_action(
        "capture_image",
        {},
        speak=speak,
        execute=execute,
        pause=pause,
    )
    assert result["offered"] is True
    assert result["announcement"] == "One moment, let me look."
    assert events == [
        ("speak", "One moment, let me look."),
        ("run", "capture_image", {}),
    ]
    assert pause.events == ["pause", "resume"]
    assert pause.paused is False


def test_a_read_tool_is_not_a_robot_action():
    pause = _Pause()
    events = []
    result = perform_robot_action(
        "list_motors",
        {},
        speak=lambda text: events.append(text),
        execute=lambda name, arguments: events.append(name) or {"ok": True},
        pause=pause,
    )
    assert result["announcement"] is None
    assert events == ["list_motors"]
    assert pause.events == []


def test_long_running_actions_are_not_offered_in_a_live_turn():
    def speak(_text):
        raise AssertionError("a long-running action must not be announced")

    def execute(_name, _arguments):
        raise AssertionError("a long-running action must not run")

    result = perform_robot_action(
        "run_program",
        {"program_id": "7"},
        speak=speak,
        execute=execute,
        pause=_Pause(),
    )
    assert result["offered"] is False
    assert result["executed"] is False
    assert result["reason"] == "long_running"

    offered = gemini_function_declarations(
        [
            {"name": "run_program", "description": "run", "parameters": {}},
            {
                "name": "capture_image",
                "description": "look",
                "parameters": {"type": "object", "properties": {}},
            },
        ]
    )
    assert [item["name"] for item in offered] == ["capture_image"]


def test_function_calls_are_read_from_the_live_tool_message():
    calls = iter_function_calls(
        {
            "function_calls": [
                {"id": "call-1", "name": "apply_pose", "args": {"pose_name": "Rest"}}
            ]
        }
    )
    assert calls == [
        {
            "id": "call-1",
            "name": "apply_pose",
            "arguments": {"pose_name": "Rest"},
        }
    ]


def test_personality_dialog_cannot_reach_the_actuation_gate(app, monkeypatch):
    monkeypatch.delenv("PIB_MCP_ENABLE_ACTUATION", raising=False)
    assert "personality" not in inspect.signature(create_server).parameters
    server = create_server()
    blocked = _call(server, "move_motor", {"motor_name": "head", "position": 0})
    assert blocked["error"]["code"] == "actuation_disabled"

    client = app.test_client()
    created = client.post(
        "/voice-assistant/personality",
        json={
            "name": "Gate",
            "gender": "Female",
            "pauseThreshold": 0.8,
            "messageHistory": 5,
            "actuationEnabled": True,
        },
    )
    assert created.status_code == 400

    created = client.post(
        "/voice-assistant/personality",
        json={
            "name": "Gate",
            "gender": "Female",
            "pauseThreshold": 0.8,
            "messageHistory": 5,
        },
    )
    assert created.status_code == 201
    body = created.get_json()
    assert "actuation" not in body
    assert "actuationEnabled" not in body
    assert personality_requests_actuation({"enableActuation": True}) is True
    assert personality_requests_actuation({"name": "Gate"}) is False

    rejected = client.put(
        f"/voice-assistant/personality/{body['personalityId']}",
        json={"mcpActuation": True},
    )
    assert rejected.status_code == 400
    blocked_again = _call(server, "apply_pose", {"pose_name": "Rest"})
    assert blocked_again["error"]["code"] == "actuation_disabled"
