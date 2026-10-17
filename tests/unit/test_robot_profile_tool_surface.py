"""The profile a robot personality gets: its tool surface and the fields it must not carry.

Two decisions of 2026-10-16, both measured on the test robot first:

* PC-70: the profile is cloned from the full Hermes configuration, so the cloud model chose from the
  whole tool zoo (browser, web, image generation, vision, terminal, the generic tool dispatcher) and
  narrated a tool call instead of making one. A robot personality sees the MCP server's tools and
  memory, nothing else.
* PC-69: the profile carried ``agent.reasoning_effort``, the Hermes client sent it with every
  request, and the pib.Cloud API refused the turn with ``400 unknown field 'reasoning_effort'``. The
  cloud decides the effort of its own upstream call, so the profile carries none.
"""

import yaml

from public_api_client.hermes_agent_client import (
    DEFAULT_MAX_TOKENS,
    DEFAULT_REASONING_EFFORT,
    DEFAULT_TEMPERATURE,
    ROBOT_TOOLSETS,
    _ensure_mcp_servers_pib,
)


def _write_profile(pdir, config):
    pdir.mkdir(exist_ok=True)
    with open(pdir / "config.yaml", "w", encoding="utf-8") as fh:
        yaml.safe_dump(config, fh)


def _read_profile(pdir):
    with open(pdir / "config.yaml", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def test_a_pib_cloud_profile_carries_no_reasoning_effort(tmp_path):
    """PC-69: the cloud owns the effort, and the field is a refused request field."""
    pdir = tmp_path / "pib_cloud"
    _write_profile(
        pdir,
        {
            "model": "pib-cloud",
            "provider": "pib-cloud",
            "agent": {"reasoning_effort": "medium", "max_turns": 4},
        },
    )

    _ensure_mcp_servers_pib(str(pdir), model="pib-cloud", provider="pib-cloud")

    cfg = _read_profile(pdir)
    assert "reasoning_effort" not in cfg["agent"]
    assert cfg["agent"]["max_turns"] == 4
    assert cfg["agent"]["max_tokens"] == DEFAULT_MAX_TOKENS
    assert cfg["agent"]["temperature"] == DEFAULT_TEMPERATURE


def test_a_personality_reasoning_level_does_not_reintroduce_it_for_pib_cloud(tmp_path):
    """PC-69: the stored level may not smuggle the field back in."""
    pdir = tmp_path / "pib_cloud_level"
    _write_profile(pdir, {"model": "pib-cloud", "provider": "pib-cloud", "agent": {}})

    _ensure_mcp_servers_pib(
        str(pdir),
        model="pib-cloud",
        provider="pib-cloud",
        reasoning_effort="high",
        personality_reasoning=True,
    )

    cfg = _read_profile(pdir)
    assert "reasoning_effort" not in cfg["agent"]


def test_a_non_cloud_profile_keeps_its_reasoning_effort(tmp_path):
    """The purge is scoped: a Gemini profile keeps the behaviour it had."""
    pdir = tmp_path / "pib_gemini"
    _write_profile(
        pdir,
        {
            "model": "gemini-3.8-flash",
            "provider": "gemini",
            "agent": {"reasoning_effort": "medium"},
        },
    )

    _ensure_mcp_servers_pib(str(pdir), model="gemini-3.8-flash", provider="gemini")

    cfg = _read_profile(pdir)
    assert cfg["agent"]["reasoning_effort"] == "medium"


def test_a_gemini_profile_without_a_level_still_gets_the_default(tmp_path):
    pdir = tmp_path / "pib_gemini_default"
    _write_profile(pdir, {"model": "gemini-3.8-flash", "provider": "gemini"})

    _ensure_mcp_servers_pib(str(pdir), model="gemini-3.8-flash", provider="gemini")

    cfg = _read_profile(pdir)
    assert cfg["agent"]["reasoning_effort"] == DEFAULT_REASONING_EFFORT


def test_the_profile_gets_only_the_robot_toolsets(tmp_path):
    """PC-70: the MCP server's tools and memory - the decision of 2026-10-16."""
    pdir = tmp_path / "pib_toolsets"
    _write_profile(pdir, {"model": "pib-cloud", "provider": "pib-cloud"})

    _ensure_mcp_servers_pib(str(pdir), model="pib-cloud", provider="pib-cloud")

    cfg = _read_profile(pdir)
    assert cfg["platform_toolsets"]["cli"] == list(ROBOT_TOOLSETS)
    # The decision of 2026-10-16: the robot's tools, memory, and the Hermes skills - nothing else.
    assert list(ROBOT_TOOLSETS) == ["pib", "memory", "skills"]


def test_a_broader_toolset_list_is_narrowed_for_the_robot_platform_only(tmp_path):
    """PC-70: a profile cloned with the full configuration is repaired, other platforms stay."""
    pdir = tmp_path / "pib_toolsets_cloned"
    _write_profile(
        pdir,
        {
            "model": "pib-cloud",
            "provider": "pib-cloud",
            "platform_toolsets": {
                "cli": ["web", "browser", "image_gen", "vision", "terminal"],
                "telegram": ["hermes-telegram"],
            },
        },
    )

    _ensure_mcp_servers_pib(str(pdir), model="pib-cloud", provider="pib-cloud")

    cfg = _read_profile(pdir)
    assert cfg["platform_toolsets"]["cli"] == list(ROBOT_TOOLSETS)
    assert cfg["platform_toolsets"]["telegram"] == ["hermes-telegram"]
