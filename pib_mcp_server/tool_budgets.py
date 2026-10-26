"""Declared UTF-8 JSON payload budgets for MCP tool results (default / compact)."""

from __future__ import annotations

import json
from typing import Any

DEFAULT_TOOL_PAYLOAD_BUDGET_BYTES = 8 * 1024

TOOL_PAYLOAD_BUDGET_BYTES: dict[str, int] = {
    "list_motors": 6 * 1024,
    "get_state": 12 * 1024,
    "list_programs": 4 * 1024,
    "list_poses": 4 * 1024,
    "capture_image": 1 * 1024,
}


def tool_payload_budget_bytes(tool_name: str) -> int:
    return TOOL_PAYLOAD_BUDGET_BYTES.get(tool_name, DEFAULT_TOOL_PAYLOAD_BUDGET_BYTES)


def payload_utf8_byte_length(payload: Any) -> int:
    return len(
        json.dumps(payload, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    )
