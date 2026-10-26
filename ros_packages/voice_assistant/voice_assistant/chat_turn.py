"""Explicit chat turn ids, cancel reasons, and turn-trace events (PR-2069)."""

from __future__ import annotations

import uuid
from dataclasses import dataclass
from typing import Callable, Optional

from voice_assistant.turn_latency import format_event

# Turn lifecycle values published on /voice_turn_trace (stage ``turn_state``).
STATE_RUNNING = "running"
STATE_CANCELLED = "cancelled"
STATE_COMPLETED = "completed"
STATE_REPLACED = "replaced"
STATE_QUEUED = "queued"

# ``SendChatMessage`` / ``CancelChatTurn`` status_reason when successful is false.
REASON_TURN_IN_PROGRESS = "turn_in_progress"
REASON_UNKNOWN_TURN = "unknown_turn_id"
REASON_ALREADY_FINISHED = "turn_already_finished"


def new_turn_id() -> str:
    """Eight hex chars, same shape as latency ``TurnTrace.turn_id``."""
    return uuid.uuid4().hex[:8]


def turn_state_event(
    turn_id: str,
    state: str,
    *,
    chat_id: str = "",
    reason: str = "",
    source: str = "chat",
) -> str:
    """JSON for one ``turn_state`` trace line."""
    detail: dict[str, str] = {"turn_id": turn_id, "state": state}
    if reason:
        detail["reason"] = reason
    return format_event(
        "turn_state",
        source=source,
        chat_id=chat_id,
        detail=detail,
    )


@dataclass
class TextTurnRecord:
    turn_id: str
    chat_id: str
    stop: Callable[[], None]
    running: bool = True


class TextTurnRegistry:
    """Tracks text-only ``send_chat_message`` turns for one assistant node."""

    def __init__(self) -> None:
        self._turns: dict[str, TextTurnRecord] = {}
        self._active_chat: dict[str, str] = {}
        self._finished: set[str] = set()

    def start(self, chat_id: str, stop: Callable[[], None]) -> str:
        active_id = self._active_chat.get(chat_id)
        if active_id:
            record = self._turns.get(active_id)
            if record is not None and record.running:
                raise TurnBusy(REASON_TURN_IN_PROGRESS)
        turn_id = new_turn_id()
        self._turns[turn_id] = TextTurnRecord(
            turn_id=turn_id, chat_id=chat_id, stop=stop, running=True
        )
        self._active_chat[chat_id] = turn_id
        return turn_id

    def finish(self, turn_id: str) -> None:
        record = self._turns.pop(turn_id, None)
        if record is None:
            return
        record.running = False
        self._finished.add(turn_id)
        if self._active_chat.get(record.chat_id) == turn_id:
            self._active_chat.pop(record.chat_id, None)

    def get(self, turn_id: str) -> Optional[TextTurnRecord]:
        return self._turns.get(turn_id)

    def cancel(self, turn_id: str) -> tuple[bool, str]:
        if turn_id in self._finished:
            return False, REASON_ALREADY_FINISHED
        record = self._turns.get(turn_id)
        if record is None:
            return False, REASON_UNKNOWN_TURN
        if not record.running:
            return False, REASON_ALREADY_FINISHED
        record.running = False
        record.stop()
        if self._active_chat.get(record.chat_id) == turn_id:
            self._active_chat.pop(record.chat_id, None)
        return True, ""


class TurnBusy(Exception):
    def __init__(self, reason: str) -> None:
        self.reason = reason
        super().__init__(reason)
