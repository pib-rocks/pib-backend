"""Register episodic memory tools on an MCP server."""

from __future__ import annotations

from typing import Any

from mcp.types import ToolAnnotations

from .recall import recall
from .store import MemoryStore

_store: MemoryStore | None = None


def _store_or_none() -> MemoryStore | None:
    """Open the store on first use. A broken path must not stop the MCP server."""
    global _store
    if _store is None:
        try:
            _store = MemoryStore.default()
        except Exception:
            return None
    return _store


def register_memory_tools(server: Any) -> None:
    read_annotations = ToolAnnotations(
        readOnlyHint=True, destructiveHint=False, idempotentHint=True
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
