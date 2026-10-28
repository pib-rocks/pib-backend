"""PR-2103: pin the MCP tool schema prompt budget for chat turns."""

from __future__ import annotations

import asyncio
import re

import pytest

from pib_mcp_server.server import ACTUATING_TOOLS, create_server
from pib_mcp_server.tool_budgets import (
    CHAT_MCP_TOOL_SCHEMA_BUDGET_CHARS,
    chat_mcp_tool_schema_prompt_chars,
)
from test_pib_mcp_server import FakeBackend

_READ_SIDE_EFFECT = re.compile(
    r"read-only|no motion|metadata only|ref/metadata",
    re.IGNORECASE,
)
_WRITE_SIDE_EFFECT = re.compile(r"side effect", re.IGNORECASE)
_UNIT_HINT = re.compile(
    r"degree|degrees|radians|0-255|1-3|500|program|pose|device|scene|motor|camera|relay|soul|smart",
    re.IGNORECASE,
)


def _tools(server):
    return asyncio.run(server.list_tools())


def test_chat_mcp_tool_schema_prompt_stays_within_budget():
    server = create_server(FakeBackend())
    tools = _tools(server)
    measured = chat_mcp_tool_schema_prompt_chars(tools)
    assert measured <= CHAT_MCP_TOOL_SCHEMA_BUDGET_CHARS, (
        f"MCP tool schema prompt is {measured} chars; "
        f"budget is {CHAT_MCP_TOOL_SCHEMA_BUDGET_CHARS}"
    )


def test_every_tool_description_states_unit_and_side_effect():
    server = create_server(FakeBackend())
    for tool in _tools(server):
        description = getattr(tool, "description", "") or ""
        assert _UNIT_HINT.search(
            description
        ), f"{tool.name} description missing a unit hint: {description!r}"
        if tool.name in ACTUATING_TOOLS or tool.name == "soul_append":
            assert _WRITE_SIDE_EFFECT.search(description), tool.name
        elif tool.name == "capture_image":
            assert _WRITE_SIDE_EFFECT.search(description), tool.name
        else:
            assert _READ_SIDE_EFFECT.search(description), tool.name


@pytest.mark.parametrize("budget", [CHAT_MCP_TOOL_SCHEMA_BUDGET_CHARS])
def test_schema_budget_constant_matches_acceptance_target(budget):
    assert budget == 3800
