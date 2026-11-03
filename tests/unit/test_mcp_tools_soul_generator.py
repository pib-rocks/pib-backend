"""Manifest-driven MCP tool prose for the generated SOUL section."""

from __future__ import annotations

import pytest

from pib_capability.manifest import load_manifest
from pib_hermes_config import (
    MCP_TOOL_NAME_PREFIX,
    MCP_TOOLS,
    MCP_TOOLS_SOUL_SECTION,
    _build_mcp_tools_soul_section,
    mcp_tool_name,
    regenerate_mcp_tools_soul_section,
    strip_mcp_tools_soul_section,
)
from pib_hermes_config.mcp_tool_text import (
    skills_missing_descriptions,
    validate_mcp_tool_descriptions,
)


def test_every_manifest_skill_has_a_description_and_no_orphans():
    validate_mcp_tool_descriptions()


def test_pib5edu_soul_section_lists_all_manifest_tools():
    manifest = load_manifest("pib5edu")
    section = _build_mcp_tools_soul_section("pib5edu")
    for skill in manifest["skills"]:
        assert f"### {mcp_tool_name(skill)}" in section
    assert len(manifest["skills"]) == 22
    assert section.count(MCP_TOOL_NAME_PREFIX) == 22


def test_skill_without_description_is_reported():
    manifest = load_manifest("pib5edu")
    extra = list(manifest["skills"]) + ["future_tool"]
    assert skills_missing_descriptions(extra) == ["future_tool"]


def test_appended_lessons_survive_mcp_section_regeneration():
    identity = "Du bist der humanoide Roboter Eva.\n"
    lesson = "Der Nutzer heißt Jürgen."
    with_section = identity + "\n" + MCP_TOOLS_SOUL_SECTION.strip() + "\n" + lesson
    refreshed = regenerate_mcp_tools_soul_section(with_section, "pib5edu")
    assert lesson in refreshed
    assert strip_mcp_tools_soul_section(refreshed) == identity + lesson + "\n"
    assert f"### {mcp_tool_name('pib_recall')}" in refreshed
    assert f"### {mcp_tool_name('list_smarthome_devices')}" in refreshed


def test_mcp_tools_tuple_matches_default_manifest():
    manifest = load_manifest("pib5edu")
    assert [tool for tool, _desc in MCP_TOOLS] == list(manifest["skills"])
