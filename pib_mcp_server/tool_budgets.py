"""Declared UTF-8 JSON payload budgets for MCP tool results (default / compact)."""

from __future__ import annotations

import json
from typing import Any

# PR-2103: chat turns inline every MCP tool schema (name + description + JSON schema).
# Measured on the test robot at develop 5c43bb46: 18 pib tools ≈ 5654 characters (~1414 tokens).
CHAT_MCP_TOOL_SCHEMA_BUDGET_CHARS = 3800

DEFAULT_TOOL_PAYLOAD_BUDGET_BYTES = 8 * 1024

TOOL_PAYLOAD_BUDGET_BYTES: dict[str, int] = {
    "list_motors": 6 * 1024,
    "get_state": 12 * 1024,
    "list_programs": 4 * 1024,
    "list_poses": 4 * 1024,
    "capture_image": 1 * 1024,
    "move_motors": 8 * 1024,
    "apply_joint_trajectory": 8 * 1024,
}


def tool_payload_budget_bytes(tool_name: str) -> int:
    return TOOL_PAYLOAD_BUDGET_BYTES.get(tool_name, DEFAULT_TOOL_PAYLOAD_BUDGET_BYTES)


def payload_utf8_byte_length(payload: Any) -> int:
    return len(
        json.dumps(payload, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    )


def tool_input_schema_dict(tool: Any) -> dict[str, Any]:
    """Normalize a FastMCP / MCP tool object to a JSON-schema mapping."""
    schema = getattr(tool, "input_schema", None)
    if schema is None:
        schema = getattr(tool, "inputSchema", None)
    if isinstance(schema, dict):
        return compact_tool_input_schema(schema)
    if hasattr(schema, "model_dump"):
        return compact_tool_input_schema(schema.model_dump())
    dumped = tool.model_dump()
    raw = dumped.get("inputSchema") or dumped.get("input_schema") or {}
    return compact_tool_input_schema(raw) if isinstance(raw, dict) else {}


def compact_tool_input_schema(schema: dict[str, Any]) -> dict[str, Any]:
    """Drop JSON Schema ``title`` keys only (parameter names and shapes unchanged)."""

    def _walk(node: Any) -> Any:
        if isinstance(node, dict):
            return {key: _walk(value) for key, value in node.items() if key != "title"}
        if isinstance(node, list):
            return [_walk(item) for item in node]
        return node

    return _walk(schema)


def chat_mcp_tool_schema_prompt_chars(tools: list[Any]) -> int:
    """Sum name + description + compact JSON schema length (PR-2103 introspection)."""
    total = 0
    for tool in tools:
        schema = tool_input_schema_dict(tool)
        description = getattr(tool, "description", None) or ""
        total += len(
            tool.name + description + json.dumps(schema, separators=(",", ":"))
        )
    return total
