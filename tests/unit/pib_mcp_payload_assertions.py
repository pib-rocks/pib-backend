"""Shared helpers for MCP tool payload budget tests."""

from __future__ import annotations

from typing import Any

from pib_mcp_server.tool_budgets import (
    payload_utf8_byte_length,
    tool_payload_budget_bytes,
)


def assert_tool_payload_within_budget(tool_name: str, payload: Any) -> int:
    """Assert compact tool JSON is within the declared byte budget; return byte length."""

    budget = tool_payload_budget_bytes(tool_name)
    nbytes = payload_utf8_byte_length(payload)
    assert (
        nbytes <= budget
    ), f"{tool_name} payload {nbytes} bytes exceeds budget {budget}"
    return nbytes
