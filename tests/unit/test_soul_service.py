"""SOUL.md materialization tests.

The old template appended a "## Verfuegbare MCP-Werkzeuge (pib_mcp_server)"
section. It is gone, and a stored text that still carries it must not reach the
file: the assertions that pinned the section are replaced by assertions that it
is absent.
"""

import os
from pib_hermes_config import (
    MCP_TOOLS,
    MCP_TOOLS_SOUL_SECTION,
    build_default_soul_text,
    strip_mcp_tools_soul_section,
)
from service.soul_service import soul_path_for, write_soul, read_soul


def _legacy_soul_with_mcp_section(
    personality_name: str, custom_description: str | None = None
) -> str:
    """A SOUL exactly as the template generated it while it advertised the tools."""
    text = build_default_soul_text(personality_name, custom_description)
    return text.rstrip("\n") + "\n\n" + MCP_TOOLS_SOUL_SECTION.strip() + "\n"


def test_soul_path_is_inside_the_personality_profile(tmp_path, monkeypatch):
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path))
    p = soul_path_for("abc-123")
    assert p == os.path.join(str(tmp_path), "pib_abc-123", "SOUL.md")


def test_soul_path_defaults_to_the_shared_mount(monkeypatch):
    monkeypatch.delenv("PIB_HERMES_PROFILES_DIR", raising=False)
    assert soul_path_for("abc-123") == (
        "/home/pib/.hermes/profiles/pib_abc-123/SOUL.md"
    )


def test_write_then_read_roundtrip(tmp_path, monkeypatch):
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path))
    written = write_soul("abc-123", "Du bist pib.")
    assert written == "Du bist pib."
    assert read_soul("abc-123") == "Du bist pib."


def test_write_blank_seeds_default_template_and_returns_content(tmp_path, monkeypatch):
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path))
    written = write_soul("abc-123", "", personality_name="Eva")
    expected = build_default_soul_text("Eva")
    assert written == expected
    assert read_soul("abc-123") == expected
    assert "Du bist der humanoide Roboter Eva." in written
    # No tool prose in a seeded SOUL; the agent reads the tools from the MCP
    # server. This replaces an assertion that pinned the removed section.
    assert "MCP-Werkzeuge" not in written
    for tool, _description in MCP_TOOLS:
        assert tool not in written


def test_write_strips_a_stored_text_that_still_carries_the_mcp_section(
    tmp_path, monkeypatch
):
    """A description saved under the old template must come out cleaned."""
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path))
    legacy = _legacy_soul_with_mcp_section("Eva", "Sei freundlich.")

    written = write_soul("abc-123", legacy, personality_name="Eva")

    assert written == build_default_soul_text("Eva", "Sei freundlich.")
    assert read_soul("abc-123") == written
    assert "MCP-Werkzeuge" not in written
    assert MCP_TOOLS_SOUL_SECTION.strip() not in written
    for tool, _description in MCP_TOOLS:
        assert tool not in written
    assert strip_mcp_tools_soul_section(written) == written


def test_read_missing_returns_empty(tmp_path, monkeypatch):
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path))
    assert read_soul("nope") == ""
