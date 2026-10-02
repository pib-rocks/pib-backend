"""What the face, the display and the LED ring show (concept sections 6.5 and 6.8).

Listening, thinking and speaking are derived from ``chat_is_listening`` and
``voice_assistant_state``. Degraded mode and a fallback answer are decided
here as well, so every surface reads one result. The holder of the voice
channel is the personality on that same state.

A gesture while an answer is spoken is an MCP tool call (``apply_pose`` or
``move_motor``). This module does not drive motors itself.
"""

from __future__ import annotations

import json
import os
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Mapping, Optional
from urllib.request import urlopen

PHASE_IDLE = "idle"
PHASE_LISTENING = "listening"
PHASE_THINKING = "thinking"
PHASE_SPEAKING = "speaking"
PHASE_DEGRADED = "degraded"
PHASE_FALLBACK = "fallback"

# Firmware patterns on the ReSpeaker ring. Idle restores the stored pattern.
LED_BY_PHASE = {
    PHASE_LISTENING: "listen",
    PHASE_THINKING: "think",
    PHASE_SPEAKING: "speak",
    PHASE_DEGRADED: "spin",
    PHASE_FALLBACK: "trace",
}

# Body language during an answer. These are existing MCP tools.
ANSWER_GESTURE_TOOLS = frozenset({"apply_pose", "move_motor"})

VOICE_USING_FALLBACK_TOPIC = "voice_using_fallback"
MODE_DEGRADED = "degraded"
MODE_UNLOCKED = "unlocked"
_KEY_STORE_PATH = "/system/key-store"


@dataclass(frozen=True)
class VisibleState:
    """One glance: the phase, who holds the voice, and how to show it."""

    phase: str
    holder_personality_id: str
    holder_name: str
    led_mode: Optional[str]
    expression: Optional[str]
    display_text: Optional[str]


def engine_is_fallback(active_backend: object) -> bool:
    """True when speech in or out had to leave its primary engine."""
    return active_backend == "fallback"


def answer_uses_fallback(text: object, fallback_reply: str) -> bool:
    """True when this spoken piece is the agent's fallback sentence.

    A clause is enough: the reply is two sentences and playback speaks them
    one at a time.
    """
    if not isinstance(text, str) or not isinstance(fallback_reply, str):
        return False
    spoken = " ".join(text.split())
    fallback = " ".join(fallback_reply.split())
    if not spoken or not fallback:
        return False
    return spoken == fallback or spoken in fallback


def issue_answer_gesture(
    tool_name: str,
    arguments: Optional[Mapping[str, Any]],
    execute: Callable[[str, dict], Any],
) -> Any:
    """Run a gesture as an MCP tool call. There is no gesture player."""
    if tool_name not in ANSWER_GESTURE_TOOLS:
        raise ValueError(
            f"{tool_name} is not a gesture during an answer; "
            "use the MCP tools apply_pose or move_motor"
        )
    payload = dict(arguments or {})
    return execute(tool_name, payload)


_mode_lock = threading.Lock()
_mode_cache: dict[str, Any] = {"value": None, "at": 0.0}
_mode_refreshing = False


def cached_key_store_mode(max_age: float = 5.0) -> Optional[str]:
    """Last key-store mode, refreshed off the caller's thread.

    The display and the microphone node run a single-threaded executor.
    A blocking read on that thread would stall the face and the direction
    of arrival, so a stale cache is returned and the refresh happens aside.
    """
    global _mode_refreshing
    now = time.monotonic()
    with _mode_lock:
        value = _mode_cache["value"]
        fresh = (now - float(_mode_cache["at"])) < max_age and _mode_cache["at"]
        if fresh or _mode_refreshing:
            return value
        _mode_refreshing = True

    def _read() -> None:
        global _mode_refreshing
        mode = read_key_store_mode(timeout=0.4)
        with _mode_lock:
            if mode is not None:
                _mode_cache["value"] = mode
            _mode_cache["at"] = time.monotonic()
            _mode_refreshing = False

    threading.Thread(target=_read, name="key-store-mode", daemon=True).start()
    return value


def read_key_store_mode(opener=None, timeout: float = 1.5) -> Optional[str]:
    """``degraded`` or ``unlocked``, or None when the store cannot be read.

    An unreadable store is not shown as degraded: the face must not claim
    the keys are locked when it could not check.
    """
    if opener is None:
        opener = urlopen
    base = os.environ.get("FLASK_API_BASE_URL", "http://127.0.0.1:5000").rstrip("/")
    try:
        with opener(base + _KEY_STORE_PATH, timeout=timeout) as response:
            body = json.loads(response.read().decode("utf-8"))
    except Exception:
        return None
    if not isinstance(body, dict):
        return None
    mode = body.get("mode")
    if mode in (MODE_DEGRADED, MODE_UNLOCKED):
        return mode
    return None


def resolve_visible_state(
    *,
    turned_on: bool,
    chat_id: str = "",
    personality_id: str = "",
    personality_name: str = "",
    listening: bool = False,
    listening_chat_id: str = "",
    speaking: bool = False,
    using_fallback: bool = False,
    operating_mode: Optional[str] = None,
) -> VisibleState:
    """The only mapping from voice state to the face, the display and the ring.

    ``listening`` counts only for the chat that holds ``voice_assistant_state``.
    Speaking wins over listening, because a live session stays open while the
    answer plays. Degraded and fallback replace the idle face, and a fallback
    answer replaces speaking, so those states come from this function too.
    """
    active_listening = bool(
        turned_on and listening and chat_id and listening_chat_id == chat_id
    )
    if not turned_on:
        phase = PHASE_IDLE
    elif speaking:
        phase = PHASE_SPEAKING
    elif active_listening:
        phase = PHASE_LISTENING
    else:
        phase = PHASE_THINKING

    if phase == PHASE_IDLE:
        if operating_mode == MODE_DEGRADED:
            phase = PHASE_DEGRADED
        elif using_fallback:
            phase = PHASE_FALLBACK
    elif phase == PHASE_SPEAKING and using_fallback:
        phase = PHASE_FALLBACK

    holder_id = ""
    holder_name = ""
    if turned_on and personality_id:
        holder_id = personality_id
        holder_name = (
            personality_name.strip() if isinstance(personality_name, str) else ""
        )

    return VisibleState(
        phase=phase,
        holder_personality_id=holder_id,
        holder_name=holder_name,
        led_mode=LED_BY_PHASE.get(phase),
        expression=None if phase == PHASE_IDLE else phase,
        display_text=_display_text(phase, holder_name, operating_mode),
    )


def _display_text(
    phase: str, holder_name: str, operating_mode: Optional[str]
) -> Optional[str]:
    if phase == PHASE_IDLE:
        return None
    lines = []
    if holder_name:
        lines.append(holder_name)
    lines.append(phase)
    if operating_mode == MODE_DEGRADED and phase != PHASE_DEGRADED:
        lines.append(PHASE_DEGRADED)
    return "\n".join(lines)
