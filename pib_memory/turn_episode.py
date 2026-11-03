"""Record one compact episodic row per completed conversation turn."""

from __future__ import annotations

import logging
import threading
from typing import Iterable

from .store import MemoryStore

logger = logging.getLogger(__name__)

_FIELD_LIMIT = 200
_recorded: set[str] = set()
_lock = threading.Lock()


def _truncate(text: str, *, limit: int = _FIELD_LIMIT) -> str:
    if len(text) <= limit:
        return text
    return text[:limit]


def _participants(participants: Iterable[str] | None) -> list[str] | None:
    if participants is None:
        return None
    names = [str(name).strip() for name in participants if str(name).strip()]
    return names or None


def record_turn(
    *,
    chat_id: str,
    turn_id: str,
    task: str,
    outcome: str | None = None,
    participants: list[str] | None = None,
    failure_reason: str | None = None,
) -> None:
    """Write one episode for a finished turn. Failures must not disturb the caller."""
    try:
        cid = (chat_id or "").strip()
        tid = (turn_id or "").strip()
        if not cid or not tid:
            return
        evidence_ref = f"{cid}#{tid}"

        task_text = _truncate((task or "").strip())
        if not task_text:
            return

        out_text = _truncate((outcome or "").strip()) if outcome else ""
        fail_text = _truncate((failure_reason or "").strip()) if failure_reason else ""
        if not out_text and not fail_text:
            return

        with _lock:
            if evidence_ref in _recorded:
                return

        store = MemoryStore.default()
        try:
            with _lock:
                if evidence_ref in _recorded:
                    return
                existing = (
                    store.connection()
                    .execute(
                        "SELECT 1 FROM episodes WHERE evidence_ref = ? LIMIT 1",
                        (evidence_ref,),
                    )
                    .fetchone()
                )
                if existing:
                    _recorded.add(evidence_ref)
                    return
                store.write_episode(
                    task=task_text,
                    outcome=out_text or None,
                    failure_reason=fail_text or None,
                    participants=_participants(participants),
                    evidence_ref=evidence_ref,
                )
                _recorded.add(evidence_ref)
        finally:
            store.close()
    except Exception as exc:
        logger.warning("turn episode was not recorded: %s", exc)
