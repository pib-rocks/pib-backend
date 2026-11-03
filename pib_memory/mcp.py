"""Register episodic memory tools on an MCP server."""

from __future__ import annotations

from typing import Any

from mcp.types import ToolAnnotations

from .recall import recall
from .store import MemoryStore


def register_memory_tools(server: Any) -> None:
    read_annotations = ToolAnnotations(
        readOnlyHint=True, destructiveHint=False, idempotentHint=True
    )
    store = MemoryStore.default()

    @server.tool(annotations=read_annotations, structured_output=True)
    def pib_recall(
        query: str,
        k: int = 5,
        person: str | None = None,
        place: str | None = None,
    ) -> dict[str, Any]:
        """Recall robot episodic memory (top-k, time-decayed, evidence-backed)."""

        records = recall(query, k=k, person=person, place=place, store=store)
        return {"ok": True, "records": records}
