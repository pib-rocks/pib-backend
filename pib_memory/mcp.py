"""Register episodic memory tools on an MCP server."""

from __future__ import annotations

import time
from typing import Any
from uuid import uuid4

from mcp.types import ToolAnnotations

from .recall import recall
from .store import MemoryStore

_store: MemoryStore | None = None

_INPUT_LIMIT = 500
_TASK_LIMIT = 200
_DEDUPE_WINDOW_MS = 30_000
_USER_REQUESTED_SCOPE = "user_requested"


def _store_or_none() -> MemoryStore | None:
    """Open the store on first use. A broken path must not stop the MCP server."""
    global _store
    if _store is None:
        try:
            _store = MemoryStore.default()
        except Exception:
            return None
    return _store


def _truncate_task(text: str) -> str:
    if len(text) <= _TASK_LIMIT:
        return text
    return text[:_TASK_LIMIT]


def _recent_user_requested_id(store: MemoryStore, task: str, now_ms: int) -> int | None:
    cutoff = now_ms - _DEDUPE_WINDOW_MS
    row = (
        store.connection()
        .execute(
            """
            SELECT id FROM episodes
            WHERE consent_scope = ? AND task = ? AND created_ms >= ?
            ORDER BY id DESC LIMIT 1
            """,
            (_USER_REQUESTED_SCOPE, task, cutoff),
        )
        .fetchone()
    )
    if row is None:
        return None
    return int(row["id"])


def register_memory_tools(server: Any) -> None:
    read_annotations = ToolAnnotations(
        readOnlyHint=True, destructiveHint=False, idempotentHint=True
    )
    write_annotations = ToolAnnotations(
        readOnlyHint=False, destructiveHint=False, idempotentHint=False
    )

    @server.tool(annotations=read_annotations, structured_output=True)
    def pib_recall(
        query: str,
        k: int = 5,
        person: str | None = None,
        place: str | None = None,
    ) -> dict[str, Any]:
        """Read-only. Recall episodic memory records (top-k, time-decayed) with date and evidence."""

        store = _store_or_none()
        if store is None:
            return {"ok": False, "reason": "memory_store_unavailable", "records": []}
        records = recall(query, k=k, person=person, place=place, store=store)
        return {"ok": True, "records": records}

    @server.tool(annotations=write_annotations, structured_output=True)
    def pib_remember(text: str) -> dict[str, Any]:
        """Side effect. Store one memory row (max 500 characters) when the user explicitly asks to remember something."""

        trimmed = (text or "").strip()
        if not trimmed:
            return {"ok": False, "reason": "text_empty"}
        if len(trimmed) > _INPUT_LIMIT:
            return {"ok": False, "reason": "text_too_long"}

        store = _store_or_none()
        if store is None:
            return {"ok": False, "reason": "memory_store_unavailable"}

        task = _truncate_task(trimmed)
        now_ms = int(time.time() * 1000)
        existing_id = _recent_user_requested_id(store, task, now_ms)
        if existing_id is not None:
            return {"ok": True, "id": existing_id, "deduplicated": True}

        row_id = store.write_episode(
            task=task,
            outcome=None,
            participants=None,
            place_id=None,
            evidence_ref=f"manual:{uuid4().hex}",
            consent_scope=_USER_REQUESTED_SCOPE,
        )
        return {"ok": True, "id": row_id}
