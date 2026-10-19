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


def test_the_mcp_server_runs_on_the_interpreter_that_has_its_dependencies():
    """PC-71: a bare "python3" resolved to the Hermes venv, which lacks websocket-client."""
    from public_api_client.hermes_agent_client import MCP_SERVER_PYTHON, PIB_MCP_SERVER

    assert MCP_SERVER_PYTHON == "/usr/bin/python3"
    assert PIB_MCP_SERVER["command"] == MCP_SERVER_PYTHON
    assert PIB_MCP_SERVER["args"] == ["-m", "pib_mcp_server"]


def test_tool_calling_switch_controls_profile_mcp_server_and_cli_toolsets(tmp_path):
    """PC-101: tool_calling=False disables mcp_servers.pib and removes pib from platform_toolsets.cli."""
    from public_api_client.hermes_agent_client import (
        ROBOT_TOOLSETS_WITHOUT_MCP,
        profile_tool_calling_enabled,
    )

    pdir = tmp_path / "pib_tools_toggle"
    _write_profile(pdir, {"model": "pib-cloud", "provider": "pib-cloud"})

    _ensure_mcp_servers_pib(
        str(pdir),
        model="pib-cloud",
        provider="pib-cloud",
        tool_calling=False,
    )
    cfg = _read_profile(pdir)
    assert cfg["mcp_servers"]["pib"]["enabled"] is False
    assert cfg["platform_toolsets"]["cli"] == list(ROBOT_TOOLSETS_WITHOUT_MCP)
    assert profile_tool_calling_enabled(str(pdir)) is False

    # Chat-time ensure_profile repair (tool_calling=None) must preserve tool_calling=False.
    _ensure_mcp_servers_pib(str(pdir))
    cfg = _read_profile(pdir)
    assert cfg["mcp_servers"]["pib"]["enabled"] is False
    assert cfg["platform_toolsets"]["cli"] == list(ROBOT_TOOLSETS_WITHOUT_MCP)
    assert profile_tool_calling_enabled(str(pdir)) is False

    # Re-enabling tool_calling=True restores mcp_servers.pib.enabled=True and ROBOT_TOOLSETS.
    _ensure_mcp_servers_pib(
        str(pdir),
        model="pib-cloud",
        provider="pib-cloud",
        tool_calling=True,
    )
    cfg = _read_profile(pdir)
    assert cfg["mcp_servers"]["pib"]["enabled"] is True
    assert cfg["platform_toolsets"]["cli"] == list(ROBOT_TOOLSETS)
    assert profile_tool_calling_enabled(str(pdir)) is True


def test_hermes_daemon_rebuilds_cached_agent_when_tool_calling_changes(
    tmp_path, monkeypatch
):
    """PC-101: Toggling tool_calling in the profile evicts the cached AIAgent and updates turn toolsets."""
    from public_api_client import hermes_daemon as hd

    pdir = tmp_path / "pib_agent_cache"
    _write_profile(pdir, {"model": "pib-cloud", "provider": "pib-cloud"})
    _ensure_mcp_servers_pib(
        str(pdir),
        model="pib-cloud",
        provider="pib-cloud",
        base_url="https://api.pib.rocks/v1",
        env_vars=("PIB_CLOUD_API_KEY",),
        tool_calling=True,
    )
    monkeypatch.setattr(hd, "_current_home_key", lambda: str(pdir))
    monkeypatch.setattr(
        "public_api_client.hermes_agent_client._current_hermes_home",
        lambda: str(pdir),
    )
    monkeypatch.setattr(hd, "_shared_session_db", lambda: None)
    hd.clear_agent_cache()

    constructed: list[dict] = []

    class _FakeAgent:
        def __init__(self, **kwargs):
            self.kwargs = kwargs
            self.closed = False
            constructed.append(kwargs)

        def close(self):
            self.closed = True

    try:
        cached_on = hd._agent_for_chat(
            "chat-1",
            "pib,vision",
            "terminal,code_execution,file,memory,session_search",
            4,
            _FakeAgent,
            "key-1",
        )
        assert len(constructed) == 1
        assert constructed[0]["enabled_toolsets"] == ["pib", "vision"]
        assert "pib" not in constructed[0]["disabled_toolsets"]

        # Flip tool_calling to False in the profile config.
        _ensure_mcp_servers_pib(
            str(pdir),
            model="pib-cloud",
            provider="pib-cloud",
            tool_calling=False,
        )
        cached_off = hd._agent_for_chat(
            "chat-1",
            "pib,vision",
            "terminal,code_execution,file,memory,session_search",
            4,
            _FakeAgent,
            "key-1",
        )
        assert cached_off is not cached_on
        assert cached_on.agent.closed is True
        assert len(constructed) == 2
        assert constructed[1]["enabled_toolsets"] == ["vision"]
        assert "pib" in constructed[1]["disabled_toolsets"]
    finally:
        hd.clear_agent_cache()


def test_update_personality_reprovisions_profile_when_tool_calling_changes(
    app_ctx, monkeypatch
):
    """PC-101: Toggling tool_calling on a personality triggers _provision_profile with the new boolean."""
    from service import personality_service

    calls: list[dict] = []

    def fake_provision(personality_id, **kwargs):
        calls.append({"personality_id": personality_id, **kwargs})
        return {"ok": True}

    monkeypatch.setattr(personality_service, "_provision_profile", fake_provision)

    personality = personality_service.create_personality(
        {"name": "SmartTools", "tool_calling": True}
    )
    assert len(calls) == 1
    assert calls[0]["tool_calling"] is True
    calls.clear()

    # Updating with the same tool_calling value does not reprovision.
    personality_service.update_personality(
        personality.personality_id, {"tool_calling": True}
    )
    assert calls == []

    # Toggling tool_calling to False reprovisions the Hermes profile.
    personality_service.update_personality(
        personality.personality_id, {"tool_calling": False}
    )
    assert len(calls) == 1
    assert calls[0]["tool_calling"] is False

    # Toggling tool_calling back to True reprovisions again.
    personality_service.update_personality(
        personality.personality_id, {"tool_calling": True}
    )
    assert len(calls) == 2
    assert calls[1]["tool_calling"] is True
