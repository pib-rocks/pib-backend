"""Live interaction quality: who owns the turn, self-hearing, robot actions.

The provider's server-side turn detection owns the live uplink. The microphone
array's hardware voice-activity signal keeps the LED ring, the face, and the
turn-based recorder. Loud audio that never trips that signal is reported; the
report does not decide when the turn ends. A live interruption stops playback,
drops the speaker tail from the uplink, and tells the model the turn was cut.
A robot action is spoken first, and the session stays paused while it runs.
A long-running action is not offered in a live turn. The actuation gate stays
the installation environment variable.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterator, Mapping, Optional

# The turn-based player already waits 0.2 s after speech so the speaker tail
# is not cut off. The live uplink uses the same window.
PLAYBACK_TAIL_EXCLUSION_SECONDS = 0.2

# Playback is written in short slices so an interruption stops on the next one.
PLAYBACK_SLICE_SECONDS = 0.02

INTERRUPTION_REPORT = (
    "The previous spoken turn was cut off. Playback stopped before it finished."
)

# A stored program does not finish when the motion starts, so it is not
# offered inside a live turn.
LONG_RUNNING_LIVE_TOOLS = frozenset({"run_program"})

# Spoken before the matching tool runs. Looking uses the concept's phrase.
SPOKEN_BRIDGES = {
    "capture_image": "One moment, let me look.",
    "move_motor": "One moment, let me move.",
    "apply_pose": "One moment, let me move.",
    "set_led": "One moment.",
    "set_relay": "One moment.",
}

# A personality payload must not carry these. The gate is PIB_MCP_ENABLE_ACTUATION.
PERSONALITY_ACTUATION_KEYS = frozenset(
    {
        "actuation",
        "actuation_enabled",
        "actuationEnabled",
        "enable_actuation",
        "enableActuation",
        "mcp_actuation",
        "mcpActuation",
        "pib_mcp_enable_actuation",
        "pibMcpEnableActuation",
    }
)

FACE_LISTENING_TEXT = "listening"


def live_turn_detection_config() -> dict[str, Any]:
    """Provider turn detection stays on. The client does not send activity marks."""
    return {
        "realtime_input_config": {
            "automatic_activity_detection": {"disabled": False},
            "activity_handling": "START_OF_ACTIVITY_INTERRUPTS",
        }
    }


def admit_live_uplink(
    *, now: float, exclude_until: float, session_paused: bool
) -> bool:
    """Whether a microphone chunk may leave for the provider.

    Hardware voice activity is not an input. It must not gate this decision.
    """
    if session_paused:
        return False
    if now < exclude_until:
        return False
    return True


def playback_exclusion_deadline(playback_ended_at: float) -> float:
    return playback_ended_at + PLAYBACK_TAIL_EXCLUSION_SECONDS


def turn_based_is_speech(
    voice_activity: Optional[bool], amplitude_silent: bool
) -> bool:
    """The array's voice-activity signal owns the turn-based recorder.

    Amplitude is only the fallback before the first sample arrives.
    """
    if voice_activity is None:
        return not amplitude_silent
    return bool(voice_activity)


# Recorder chunks, not DOA polls. The streamer publishes 1024 frames, which is
# 64 ms at 16 kHz, so this run is about two seconds: past one buffer, and
# still inside the 8 s max_silent_seconds_before window.
ARRAY_VAD_SILENT_CHUNKS = 32

ARRAY_VAD_SILENT_WARNING = (
    "Captured audio is loud while the hardware voice-activity signal has "
    f"reported false for the last {ARRAY_VAD_SILENT_CHUNKS} chunks; the "
    "recorder counts that as silence and can end the turn on the fixed "
    "pre-speech timeout. This is not a fault by itself: the audio may be the "
    "robot's own playback, which the array's echo canceller can keep out of "
    "the detectors on purpose. It is a fault when the telemetry is unavailable "
    "or stale - on the test robot the array's control reads returned "
    "'[Errno 32] Pipe error' and the device then left the USB bus. Verify that "
    "the array still answers before reading anything into this line."
)

ARRAY_VAD_SILENT_CLEARED = (
    "array voice activity returned; the loud-audio voice-activity warning cleared"
)


@dataclass(frozen=True)
class ArrayVadWarningState:
    """Rate limit for the loud-audio, voice-activity-false diagnostic.

    ``active`` is whether that warning is already in effect. ``loud_false_chunks``
    is how many of the latest chunks were loud while voice activity was false.
    """

    active: bool = False
    loud_false_chunks: int = 0


def array_vad_silent_warning(
    voice_activity: Optional[bool],
    amplitude_silent: bool,
    state: ArrayVadWarningState,
) -> tuple[Optional[str], ArrayVadWarningState]:
    """What to log when loud audio never trips the array's voice activity.

    ``voice_activity`` is the newest sample (``None`` before the first one).
    ``amplitude_silent`` is the chunk's amplitude verdict. ``state.active`` is
    whether a warning is already active. The text is one line, or ``None``
    when nothing should be logged. The returned state is the rate limit: one
    warning per run of ``ARRAY_VAD_SILENT_CHUNKS`` loud chunks with voice
    activity false, then one line when voice activity returns.

    No sample yet is the amplitude fallback and does not warn. A quiet chunk
    does not warn and does not clear an active warning.
    """
    if voice_activity is None:
        return None, state

    if voice_activity:
        cleared = ARRAY_VAD_SILENT_CLEARED if state.active else None
        return cleared, ArrayVadWarningState()

    if amplitude_silent:
        return None, ArrayVadWarningState(active=state.active)

    streak = state.loud_false_chunks + 1
    if not state.active and streak >= ARRAY_VAD_SILENT_CHUNKS:
        return (
            ARRAY_VAD_SILENT_WARNING,
            ArrayVadWarningState(active=True, loud_false_chunks=streak),
        )
    return None, ArrayVadWarningState(active=state.active, loud_false_chunks=streak)


def face_for_hardware_vad(voice_activity: bool) -> Optional[str]:
    """Text the face shows while the array hears speech. None restores the eyes."""
    if voice_activity:
        return FACE_LISTENING_TEXT
    return None


def interruption_detected(server_content: object) -> bool:
    return bool(getattr(server_content, "interrupted", None))


def on_interruption(now: float) -> dict[str, Any]:
    """Stop playback, drop what is queued, and the text to send to the model."""
    return {
        "stop_playback": True,
        "drop_queue": True,
        "exclude_until": playback_exclusion_deadline(now),
        "report": INTERRUPTION_REPORT,
    }


def typed_text_joins_live(
    *,
    live_open: bool,
    live_chat_id: object,
    message_chat_id: object,
    text: object,
) -> dict[str, Any]:
    """A typed line for the open live chat joins that session.

    Playback stops, so the spoken turn stays interruptible, and the text is
    sent in. The session stays open. Speech barge-in stays the provider's
    own turn detection. A line for another chat is left on the ordinary
    text path. Blank text is ignored.
    """
    cleaned = text.strip() if isinstance(text, str) else ""
    same_chat = bool(live_open and live_chat_id and message_chat_id == live_chat_id)
    if same_chat and cleaned:
        return {
            "join": True,
            "interrupt": True,
            "text": cleaned,
            "keep_session": True,
        }
    return {
        "join": False,
        "interrupt": False,
        "text": None,
        "keep_session": bool(live_open),
    }


def playback_slice_bytes(
    sample_rate: int = 24000, sample_width: int = 2, channels: int = 1
) -> int:
    samples = max(1, int(sample_rate * PLAYBACK_SLICE_SECONDS))
    return samples * sample_width * channels


def iter_playback_slices(pcm: bytes, slice_bytes: int) -> Iterator[bytes]:
    if not pcm:
        return
    step = slice_bytes if slice_bytes > 0 else len(pcm)
    for offset in range(0, len(pcm), step):
        yield pcm[offset : offset + step]


def consume_playback(
    pcm: bytes, *, slice_bytes: int, cancel_after_slices: Optional[int]
) -> dict[str, Any]:
    """Bytes written before an interruption, and the tail that is dropped."""
    written: list[bytes] = []
    dropped: list[bytes] = []
    stop = False
    for index, chunk in enumerate(iter_playback_slices(pcm, slice_bytes)):
        if cancel_after_slices is not None and index >= cancel_after_slices:
            stop = True
        if stop:
            dropped.append(chunk)
        else:
            written.append(chunk)
    return {
        "written": b"".join(written),
        "dropped": b"".join(dropped),
        "stopped_early": bool(dropped),
    }


def spoken_bridge(tool_name: str) -> Optional[str]:
    return SPOKEN_BRIDGES.get(tool_name)


def gemini_function_declarations(
    declarations: list[Mapping[str, Any]],
) -> list[dict[str, Any]]:
    """Tool list for a live session. Long-running actions are left out."""
    offered: list[dict[str, Any]] = []
    for item in declarations:
        name = item.get("name")
        if not name or name in LONG_RUNNING_LIVE_TOOLS:
            continue
        parameters = item.get("parameters")
        if not isinstance(parameters, dict) or parameters.get("type") != "object":
            parameters = {"type": "object", "properties": {}}
        offered.append(
            {
                "name": str(name),
                "description": item.get("description") or "",
                "parameters": parameters,
            }
        )
    return offered


def perform_robot_action(
    tool_name: str,
    arguments: Mapping[str, Any],
    *,
    speak,
    execute,
    pause,
) -> dict[str, Any]:
    """Announce a robot action, keep the session paused until it finishes, then return.

    ``run_program`` is refused: it is not offered inside a live turn.
    A read tool runs immediately, with no announcement and no pause.
    """
    if tool_name in LONG_RUNNING_LIVE_TOOLS:
        return {
            "offered": False,
            "executed": False,
            "announcement": None,
            "result": None,
            "reason": "long_running",
        }
    bridge = spoken_bridge(tool_name)
    if bridge is None:
        result = execute(tool_name, arguments)
        return {
            "offered": True,
            "executed": True,
            "announcement": None,
            "result": result,
            "reason": None,
        }
    pause.begin()
    try:
        speak(bridge)
        result = execute(tool_name, arguments)
    finally:
        pause.end()
    return {
        "offered": True,
        "executed": True,
        "announcement": bridge,
        "result": result,
        "reason": None,
    }


def personality_requests_actuation(payload: Mapping[str, Any]) -> bool:
    return any(key in payload for key in PERSONALITY_ACTUATION_KEYS)


def iter_function_calls(tool_call: object) -> list[dict[str, Any]]:
    """Normalize a Gemini live tool-call message into plain dicts."""
    calls = getattr(tool_call, "function_calls", None)
    if calls is None and isinstance(tool_call, Mapping):
        calls = tool_call.get("function_calls") or tool_call.get("functionCalls") or []
    normalized: list[dict[str, Any]] = []
    for call in calls or []:
        if isinstance(call, Mapping):
            name = call.get("name")
            raw_args = call.get("args")
            if raw_args is None:
                raw_args = call.get("arguments")
            call_id = call.get("id")
        else:
            name = getattr(call, "name", None)
            raw_args = getattr(call, "args", None)
            call_id = getattr(call, "id", None)
        if not name:
            continue
        normalized.append(
            {
                "id": call_id or str(name),
                "name": str(name),
                "arguments": _as_dict(raw_args),
            }
        )
    return normalized


def _as_dict(value: object) -> dict[str, Any]:
    if value is None:
        return {}
    if isinstance(value, dict):
        return dict(value)
    if hasattr(value, "model_dump"):
        dumped = value.model_dump()
        return dumped if isinstance(dumped, dict) else {}
    if hasattr(value, "items"):
        return dict(value)
    return {}
