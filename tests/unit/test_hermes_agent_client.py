from public_api_client.hermes_agent_client import (
    DEFAULT_REASONING_EFFORT,
    FALLBACK_REPLY,
    PIB_CLOUD_PROVIDER,
    _ensure_mcp_servers_pib,
    apply_provider_capability_cache_to_cfg,
    build_default_soul_text,
    clear_provider_refused_fields_cache,
    ensure_profile,
    record_provider_refused_fields,
    session_name_for,
    build_command,
    hermes_binary_available,
    profile_name_for,
    provider_refuses_field,
    run_turn,
    uses_hermes_backend,
)
import yaml
from pib_hermes_config import (
    HERMES_VISION_ANALYZE_MEASURED_OUTPUT_TOKENS,
    HERMES_VISION_ANALYZE_PROMPT_PREFIX,
    HERMES_VISION_TOOLS_MODULE,
    MCP_TOOLS,
    MCP_TOOLS_SOUL_SECTION,
    PROFILE_TOOL_SEARCH_CONFIG,
    strip_mcp_tools_soul_section,
)
from public_api_client.hermes_agent_client import _ensure_mcp_servers_pib
import os
import subprocess
from unittest.mock import patch

# The MCP tool names as the module publishes them. These must NOT appear in a
# generated SOUL any more; they are asserted against the removed section below.
EXPECTED_MCP_TOOLS = (
    "mcp__pib__list_motors",
    "mcp__pib__get_motion_model",
    "mcp__pib__get_state",
    "mcp__pib__list_poses",
    "mcp__pib__list_programs",
    "mcp__pib__capture_image",
    "mcp__pib__move_motor",
    "mcp__pib__apply_pose",
    "mcp__pib__run_program",
    "mcp__pib__set_led",
    "mcp__pib__set_relay",
    "mcp__pib__soul_append",
)


def _legacy_soul_with_mcp_section(
    personality_name: str, custom_description: str | None = None
) -> str:
    """A SOUL exactly as the template generated it while it advertised the tools."""
    text = build_default_soul_text(personality_name, custom_description)
    return text.rstrip("\n") + "\n\n" + MCP_TOOLS_SOUL_SECTION.strip() + "\n"


def test_session_name_is_prefixed_and_sanitized():
    assert session_name_for("abc-123") == "pib_chat_abc-123"


def test_session_name_strips_unsafe_chars():
    assert session_name_for("a/b c!") == "pib_chat_ab_c"


def test_uses_hermes_backend_for_hermes_api_name():
    assert uses_hermes_backend("hermes-agent") is True
    assert uses_hermes_backend("gpt-6") is False


def test_profile_name_is_derived_from_personality():
    assert profile_name_for("abc-123") == "pib_abc-123"


def test_build_command_uses_oneshot_named_session_and_profile():
    cmd = build_command("hallo", "chat-1", personality_id="p-9")
    assert cmd[0].endswith("hermes")
    # The `chat` subcommand carries the turn: only it accepts --create-if-missing.
    assert cmd[1:3] == ["-p", "pib_p-9"]
    assert "chat" in cmd
    assert "-q" in cmd and "hallo" in cmd
    assert "--oneshot" in cmd and "-Q" in cmd
    assert "--continue" in cmd and "pib_chat_chat-1" in cmd  # durable per-chat session
    assert (
        "--create-if-missing" in cmd
    ), "a chat's first turn has no session to continue"


def test_build_command_without_personality_omits_profile():
    cmd = build_command("hallo", "chat-1", personality_id=None)
    assert "-p" not in cmd


def test_build_command_carries_explicit_toolsets():
    cmd = build_command(
        "hallo",
        "chat-1",
        toolsets="terminal,code_execution,file",
    )
    index = cmd.index("-t")
    assert cmd[index + 1] == "terminal,code_execution,file"


def test_build_default_soul_text_starts_with_robot_identity():
    text = build_default_soul_text("Eva")
    assert text.startswith("Du bist der humanoide Roboter Eva.")


def test_build_default_soul_text_substitutes_personality_name():
    text = build_default_soul_text("Thomas")
    assert "Du bist der humanoide Roboter Thomas." in text
    assert "{personality_name}" not in text


def test_build_default_soul_text_includes_custom_description():
    text = build_default_soul_text(
        "Eva", custom_description="Sei freundlich und neugierig."
    )
    assert text.startswith("Du bist der humanoide Roboter Eva.")
    assert "Sei freundlich und neugierig." in text


def test_build_default_soul_text_carries_no_tool_prose():
    """The SOUL must not advertise the MCP tools any more.

    This replaces an assertion that pinned the generated
    "## Verfuegbare MCP-Werkzeuge (pib_mcp_server)" section: tool prose in the
    SOUL made the model talk about its tools or write the calls out as text,
    and the agent gets their real names and schemas from the MCP server itself.
    """
    text = build_default_soul_text("pib")
    assert "MCP-Werkzeuge" not in text
    assert MCP_TOOLS_SOUL_SECTION.strip() not in text
    for tool in EXPECTED_MCP_TOOLS:
        assert tool not in text
    for tool, _description in MCP_TOOLS:
        assert tool not in text


def test_build_default_soul_text_keeps_identity_and_custom_text():
    text = build_default_soul_text("Eva", custom_description="Sei neugierig.")
    assert text == "Du bist der humanoide Roboter Eva.\n\nSei neugierig.\n"


def test_build_default_soul_text_strips_a_custom_text_that_carries_the_section():
    """Every caller folds its text in here, the daemon rebuild included."""
    text = build_default_soul_text("Eva", custom_description=MCP_TOOLS_SOUL_SECTION)
    assert text == "Du bist der humanoide Roboter Eva.\n"


def test_strip_mcp_tools_soul_section_leaves_a_text_without_the_section_alone():
    text = "Du bist der humanoide Roboter pib.\n\nSei freundlich.\n"
    assert strip_mcp_tools_soul_section(text) == text


def test_strip_mcp_tools_soul_section_removes_the_section_at_the_end():
    assert (
        strip_mcp_tools_soul_section(_legacy_soul_with_mcp_section("Eva"))
        == "Du bist der humanoide Roboter Eva.\n"
    )


def test_strip_mcp_tools_soul_section_removes_a_section_in_the_middle():
    """The custom text in front of it is not cut, and the text behind it stays."""
    legacy = _legacy_soul_with_mcp_section("Eva", "Sei freundlich.")
    text = legacy.rstrip("\n") + "\n\nNach dem Werkzeugblock.\n"
    assert strip_mcp_tools_soul_section(text) == (
        "Du bist der humanoide Roboter Eva.\n\nSei freundlich."
        "\n\nNach dem Werkzeugblock.\n"
    )


def test_strip_mcp_tools_soul_section_is_idempotent():
    once = strip_mcp_tools_soul_section(_legacy_soul_with_mcp_section("Eva"))
    assert strip_mcp_tools_soul_section(once) == once


def test_strip_mcp_tools_soul_section_recognises_a_transliterated_heading():
    """A copy made through tooling that drops the umlaut names the same block."""
    legacy = _legacy_soul_with_mcp_section("Eva").replace(
        "## Verfügbare MCP-Werkzeuge (pib_mcp_server)",
        "## Verfuegbare MCP-Werkzeuge (pib_mcp_server)",
    )
    assert strip_mcp_tools_soul_section(legacy) == (
        "Du bist der humanoide Roboter Eva.\n"
    )


def test_build_default_soul_text_defaults_blank_name_to_pib():
    text = build_default_soul_text("  ")
    assert text.startswith("Du bist der humanoide Roboter pib.")


def test_ensure_profile_writes_templated_soul_with_name_and_no_mcp_docs(
    tmp_path, monkeypatch
):
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path / "profiles"))
    monkeypatch.setenv("PIB_HERMES_PROFILE_FACTORY", "filesystem")
    monkeypatch.setenv("PIB_HERMES_BIN", str(tmp_path / "not-installed" / "hermes"))

    with patch("public_api_client.hermes_agent_client.subprocess.run") as run:
        pdir = ensure_profile(
            "p-9",
            soul_text="Sei neugierig.",
            personality_name="Eva",
        )

    run.assert_not_called()
    with open(os.path.join(pdir, "SOUL.md"), encoding="utf-8") as fh:
        content = fh.read()
    assert content.startswith("Du bist der humanoide Roboter Eva.")
    assert "Sei neugierig." in content
    # The provisioned SOUL no longer documents the tools; the agent reads them
    # from the MCP server.
    assert "MCP-Werkzeuge" not in content
    for tool in EXPECTED_MCP_TOOLS:
        assert tool not in content
    assert content == build_default_soul_text("Eva", "Sei neugierig.")


def test_ensure_profile_defaults_personality_name_to_pib(tmp_path, monkeypatch):
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path / "profiles"))
    monkeypatch.setenv("PIB_HERMES_PROFILE_FACTORY", "filesystem")
    monkeypatch.setenv("PIB_HERMES_BIN", str(tmp_path / "not-installed" / "hermes"))

    pdir = ensure_profile("p-9", soul_text="Hallo.")
    with open(os.path.join(pdir, "SOUL.md"), encoding="utf-8") as fh:
        content = fh.read()
    assert content.startswith("Du bist der humanoide Roboter pib.")
    assert "Hallo." in content


def test_run_turn_returns_stdout(installed_hermes_bin, monkeypatch):
    # Force subprocess path: no warm daemon on this port.
    monkeypatch.setenv("PIB_HERMES_DAEMON_URL", "http://127.0.0.1:1")
    completed = subprocess.CompletedProcess(
        args=[], returncode=0, stdout="Hallo!\n", stderr=""
    )
    with patch("subprocess.run", return_value=completed):
        assert run_turn("hi", "c1") == "Hallo!"


def test_subprocess_fallback_uses_voice_whitelist_not_blacklist(
    installed_hermes_bin,
):
    from public_api_client.hermes_agent_client import (
        DEFAULT_ENABLED_TOOLSETS,
        run_turn_subprocess,
    )

    completed = subprocess.CompletedProcess(
        args=[], returncode=0, stdout="Hallo!\n", stderr=""
    )
    with patch("subprocess.run", return_value=completed) as run:
        assert run_turn_subprocess("hi", "c1") == "Hallo!"

    command = run.call_args.args[0]
    assert command[command.index("-t") + 1] == DEFAULT_ENABLED_TOOLSETS


def test_run_turn_on_timeout_returns_fallback(installed_hermes_bin, monkeypatch):
    monkeypatch.setenv("PIB_HERMES_DAEMON_URL", "http://127.0.0.1:1")
    with patch(
        "subprocess.run", side_effect=subprocess.TimeoutExpired(cmd="x", timeout=1)
    ):
        out = run_turn("hi", "c1")
        assert out  # non-empty graceful sentence
        assert "moment" in out.lower() or "später" in out.lower()


def test_run_turn_on_error_returns_fallback(installed_hermes_bin, monkeypatch):
    monkeypatch.setenv("PIB_HERMES_DAEMON_URL", "http://127.0.0.1:1")
    completed = subprocess.CompletedProcess(
        args=[], returncode=1, stdout="", stderr="boom"
    )
    with patch("subprocess.run", return_value=completed):
        assert run_turn("hi", "c1")


def test_run_turn_without_installed_binary_returns_fallback(tmp_path, monkeypatch):
    monkeypatch.setenv("PIB_HERMES_BIN", str(tmp_path / "not-installed" / "hermes"))
    monkeypatch.setenv("PIB_HERMES_DAEMON_URL", "http://127.0.0.1:1")

    with patch("public_api_client.hermes_agent_client.subprocess.run") as run:
        out = run_turn("hi", "c1")

    assert out == FALLBACK_REPLY
    run.assert_not_called()  # never attempt a subprocess we know cannot start


def test_run_turn_treats_a_non_executable_binary_as_missing(tmp_path, monkeypatch):
    binary = tmp_path / "hermes"
    binary.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
    binary.chmod(0o644)
    monkeypatch.setenv("PIB_HERMES_BIN", str(binary))
    monkeypatch.setenv("PIB_HERMES_DAEMON_URL", "http://127.0.0.1:1")

    with patch("public_api_client.hermes_agent_client.subprocess.run") as run:
        assert run_turn("hi", "c1") == FALLBACK_REPLY

    run.assert_not_called()


def test_run_turn_prefers_daemon_reply_over_subprocess(
    installed_hermes_bin, monkeypatch
):
    monkeypatch.setenv("PIB_HERMES_DAEMON_URL", "http://127.0.0.1:8088")

    with patch(
        "public_api_client.hermes_agent_client._try_daemon_turn",
        return_value="from-daemon",
    ) as daemon:
        with patch("public_api_client.hermes_agent_client.subprocess.run") as run:
            assert run_turn("hi", "c1", personality_id="p-1") == "from-daemon"

    daemon.assert_called_once()
    run.assert_not_called()


def test_run_turn_falls_back_to_subprocess_when_daemon_unreachable(
    installed_hermes_bin, monkeypatch
):
    monkeypatch.setenv("PIB_HERMES_DAEMON_URL", "http://127.0.0.1:1")
    completed = subprocess.CompletedProcess(
        args=[], returncode=0, stdout="via-subprocess\n", stderr=""
    )
    with patch("subprocess.run", return_value=completed) as run:
        assert run_turn("hi", "c1") == "via-subprocess"
    run.assert_called_once()


def test_run_turn_falls_back_when_daemon_returns_non_200(
    installed_hermes_bin, monkeypatch
):
    monkeypatch.setenv("PIB_HERMES_DAEMON_URL", "http://127.0.0.1:8088/turn")

    class _Resp:
        status_code = 503

        def json(self):
            return {"error": "busy"}

    with patch("requests.post", return_value=_Resp()):
        completed = subprocess.CompletedProcess(
            args=[], returncode=0, stdout="fallback-ok\n", stderr=""
        )
        with patch("subprocess.run", return_value=completed):
            assert run_turn("hi", "c1") == "fallback-ok"


def test_hermes_binary_available_reflects_the_configured_path(
    installed_hermes_bin, monkeypatch
):
    assert hermes_binary_available() is True

    monkeypatch.setenv("PIB_HERMES_BIN", str(installed_hermes_bin) + "-gone")
    assert hermes_binary_available() is False


def test_delete_session_invokes_hermes_sessions_delete():
    from public_api_client.hermes_agent_client import delete_session

    completed = subprocess.CompletedProcess(args=[], returncode=0, stdout="", stderr="")
    with patch("subprocess.run", return_value=completed) as run:
        assert delete_session("abc-123") is True
        args = run.call_args.args[0]
        assert args[-2:] == ["delete", "pib_chat_abc-123"]
        assert "sessions" in args


def test_default_timeout_reads_pib_hermes_timeout_env(monkeypatch):
    monkeypatch.setenv("PIB_HERMES_TIMEOUT", "77")
    # Ambient FLASK_API_BASE_URL (e.g. from bricklet tests) must not rewrite
    # PIB_MCP_SERVER on reload — sibling profile tests compare against the
    # collection-time constant.
    monkeypatch.delenv("FLASK_API_BASE_URL", raising=False)
    import importlib
    import public_api_client.hermes_agent_client as hac

    importlib.reload(hac)
    try:
        assert hac.DEFAULT_TIMEOUT_SECONDS == 77
    finally:
        monkeypatch.delenv("PIB_HERMES_TIMEOUT", raising=False)
        importlib.reload(hac)


def test_voice_defaults_are_configurable_and_budget_defaults_to_four(monkeypatch):
    import importlib
    import public_api_client.hermes_agent_client as hac

    # Keep the module-level MCP fixture stable when earlier tests leave an
    # ambient local API URL behind.
    monkeypatch.delenv("FLASK_API_BASE_URL", raising=False)
    monkeypatch.setenv("PIB_HERMES_ENABLED_TOOLSETS", "mcp-pib,vision,custom")
    monkeypatch.setenv("PIB_HERMES_DISABLED_TOOLSETS", "terminal,file")
    monkeypatch.setenv("PIB_HERMES_MAX_TURNS", "6")
    importlib.reload(hac)
    try:
        assert hac.DEFAULT_ENABLED_TOOLSETS == "mcp-pib,vision,custom"
        assert hac.DEFAULT_DISABLED_TOOLSETS == "terminal,file"
        assert hac.DEFAULT_MAX_TURNS == 6
    finally:
        monkeypatch.delenv("PIB_HERMES_ENABLED_TOOLSETS", raising=False)
        monkeypatch.delenv("PIB_HERMES_DISABLED_TOOLSETS", raising=False)
        monkeypatch.delenv("PIB_HERMES_MAX_TURNS", raising=False)
        monkeypatch.delenv("FLASK_API_BASE_URL", raising=False)
        importlib.reload(hac)

    assert hac.DEFAULT_MAX_TURNS == 4
    # Bare `pib`, not `mcp-pib`: only the bare server name is registered before
    # MCP discovery, and the alias makes Hermes warn on stdout.
    assert set(hac.DEFAULT_ENABLED_TOOLSETS.split(",")) >= {"pib", "vision"}
    assert set(hac.DEFAULT_DISABLED_TOOLSETS.split(",")) >= {
        "terminal",
        "code_execution",
        "file",
        "memory",
        "session_search",
    }


def test_child_environment_sets_only_the_named_keys(monkeypatch):
    """PR-1930b: a turn keys the child under its provider's own variables."""
    from public_api_client.hermes_agent_client import _child_environment

    env = _child_environment("secret", ("OPENAI_API_KEY",))
    assert env["OPENAI_API_KEY"] == "secret"
    assert "GOOGLE_API_KEY" not in env
    assert "GEMINI_API_KEY" not in env


def test_child_environment_is_none_for_a_keyless_provider():
    from public_api_client.hermes_agent_client import _child_environment

    assert _child_environment(None, ("OPENAI_API_KEY",)) is None
    assert _child_environment("secret", ()) is None


def test_ensure_profile_writes_the_supplied_model_and_provider(tmp_path, monkeypatch):
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path / "profiles"))
    monkeypatch.setenv("PIB_HERMES_PROFILE_FACTORY", "filesystem")
    monkeypatch.setenv("PIB_HERMES_BIN", str(tmp_path / "not-installed" / "hermes"))

    pdir = ensure_profile(
        "p-local",
        soul_text="Du bist pib.",
        model="qwen-fast",
        provider="Local",
        base_url="http://host.docker.internal:11434/v1",
    )

    import yaml

    with open(os.path.join(pdir, "config.yaml"), encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)
    assert cfg["model"] == "qwen-fast"
    assert cfg["provider"] == "local"
    assert cfg["providers"]["local"]["base_url"] == (
        "http://host.docker.internal:11434/v1"
    )


def _assert_profile_keeps_robot_tools_eager(cfg):
    """PR-2053: MCP tools must not sit behind tool_search (extra model round trip)."""
    assert cfg["tools"]["tool_search"] == PROFILE_TOOL_SEARCH_CONFIG["tool_search"]


def test_profile_config_disables_tool_search_for_tool_calling(tmp_path, monkeypatch):
    """PR-2053: tool_calling=True keeps the robot MCP tools eager in config.yaml."""
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path / "profiles"))
    monkeypatch.setenv("PIB_HERMES_PROFILE_FACTORY", "filesystem")
    monkeypatch.setenv("PIB_HERMES_BIN", str(tmp_path / "not-installed" / "hermes"))

    import yaml

    pdir = ensure_profile(
        "p-tools-on",
        soul_text="Du bist pib.",
        model="gemini-3.8-flash",
        provider="gemini",
        tool_calling=True,
    )
    with open(os.path.join(pdir, "config.yaml"), encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)
    _assert_profile_keeps_robot_tools_eager(cfg)


def test_profile_config_disables_tool_search_when_tool_calling_is_off(
    tmp_path, monkeypatch
):
    """PR-2053: tool_calling=False still writes the same eager-tools knob.

    Without MCP the profile exposes memory and skills only, but Hermes would
    still defer plugin tools when tool_search stays on auto; the backend pins
    the setting for every personality it provisions.
    """
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path / "profiles"))
    monkeypatch.setenv("PIB_HERMES_PROFILE_FACTORY", "filesystem")
    monkeypatch.setenv("PIB_HERMES_BIN", str(tmp_path / "not-installed" / "hermes"))

    import yaml

    pdir = ensure_profile(
        "p-tools-off",
        soul_text="Du bist pib.",
        model="gemini-3.8-flash",
        provider="gemini",
        tool_calling=False,
    )
    with open(os.path.join(pdir, "config.yaml"), encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)
    _assert_profile_keeps_robot_tools_eager(cfg)
    assert cfg["mcp_servers"]["pib"]["enabled"] is False


def test_reprovisioning_keeps_tool_search_disabled(tmp_path):
    """PR-2053: a second _ensure_mcp_servers_pib pass must not drop the setting."""
    import yaml

    pdir = tmp_path / "pib_reprovision"
    pdir.mkdir()
    (pdir / "config.yaml").write_text(
        "model: gemini-3.8-flash\nprovider: gemini\n", encoding="utf-8"
    )
    for _ in range(2):
        _ensure_mcp_servers_pib(
            str(pdir), model="gemini-3.8-flash", provider="gemini", tool_calling=True
        )
    with open(pdir / "config.yaml", encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)
    _assert_profile_keeps_robot_tools_eager(cfg)


def test_vision_analyze_limitation_is_documented_in_pib_hermes_config():
    """PR-2053: upstream vision wrapper cost is recorded with its package path."""
    assert HERMES_VISION_TOOLS_MODULE == "tools/vision_tools.py"
    assert HERMES_VISION_ANALYZE_MEASURED_OUTPUT_TOKENS == 1444
    assert "Fully describe and explain everything about this image" in (
        HERMES_VISION_ANALYZE_PROMPT_PREFIX
    )


def test_pib_cloud_profile_written_without_reasoning_effort(tmp_path):
    """pib.Cloud refuses reasoning_effort on the wire; the profile must not carry it."""
    pdir = tmp_path / "cloud"
    pdir.mkdir()
    with open(pdir / "config.yaml", "w", encoding="utf-8") as fh:
        yaml.safe_dump(
            {
                "model": "pib-cloud",
                "provider": PIB_CLOUD_PROVIDER,
                "agent": {"reasoning_effort": "medium"},
            },
            fh,
        )

    _ensure_mcp_servers_pib(str(pdir), model="pib-cloud", provider=PIB_CLOUD_PROVIDER)

    with open(pdir / "config.yaml", encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)
    assert "reasoning_effort" not in cfg["agent"]


def test_gemini_profile_keeps_default_reasoning_effort(tmp_path):
    pdir = tmp_path / "gemini"
    pdir.mkdir()
    with open(pdir / "config.yaml", "w", encoding="utf-8") as fh:
        yaml.safe_dump({"model": "gemini-3.8-flash", "provider": "gemini"}, fh)

    _ensure_mcp_servers_pib(str(pdir), model="gemini-3.8-flash", provider="gemini")

    with open(pdir / "config.yaml", encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)
    assert cfg["agent"]["reasoning_effort"] == DEFAULT_REASONING_EFFORT


def test_recorded_refusal_omits_reasoning_effort_on_the_next_profile_write(tmp_path):
    clear_provider_refused_fields_cache()
    record_provider_refused_fields(
        "gemini", None, "gemini-3.8-flash", "reasoning_effort"
    )

    pdir = tmp_path / "gemini_refused"
    pdir.mkdir()
    with open(pdir / "config.yaml", "w", encoding="utf-8") as fh:
        yaml.safe_dump({"model": "gemini-3.8-flash", "provider": "gemini"}, fh)

    _ensure_mcp_servers_pib(str(pdir), model="gemini-3.8-flash", provider="gemini")

    with open(pdir / "config.yaml", encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)
    assert "reasoning_effort" not in cfg["agent"]
    assert provider_refuses_field(
        "gemini", None, "gemini-3.8-flash", "reasoning_effort"
    )
    clear_provider_refused_fields_cache()


def test_pib_cloud_refusals_are_known_before_the_first_request():
    """PR-2055: the guarded route must not be probed with a 400 after every start.

    Measured on .164 on 2026-10-25: the first chat after a daemon start waited
    8.2 s for its first token (agent 2.5 s + a rejected reasoning_effort and
    stream_options), while the next chat answered in 815 ms.
    """
    for field in ("reasoning_effort", "stream_options", "response_format"):
        assert provider_refuses_field("pib-cloud", None, "pib-cloud", field)
    assert not provider_refuses_field(
        "gemini", None, "gemini-3.8-flash", "reasoning_effort"
    )


def test_profile_keeps_the_derived_title_without_a_model_call(tmp_path):
    """PR-2058: no auxiliary title request, so no rejected fields per new chat.

    Measured on .164 on 2026-10-25: every new chat logged two 400s from title
    generation (reasoning_effort, response_format) after the answer was delivered.
    """
    pdir = tmp_path / "titles"
    pdir.mkdir()
    (pdir / "config.yaml").write_text(
        "model: pib-cloud\nprovider: pib-cloud\n", encoding="utf-8"
    )

    _ensure_mcp_servers_pib(str(pdir), model="pib-cloud", provider="pib-cloud")

    import yaml

    with open(pdir / "config.yaml", encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)
    assert cfg["auxiliary"]["title_generation"]["model_upgrade_enabled"] is False


def test_provisioning_overrides_an_explicit_title_upgrade_setting(tmp_path):
    """Provisioning forces the derived title even when a profile says otherwise.

    The upgrade costs a rejected request per new chat on pib.Cloud and buys only a
    prettier title, so the profile writer owns this key (PR-2058).
    """
    import yaml

    pdir = tmp_path / "titles-kept"
    pdir.mkdir()
    (pdir / "config.yaml").write_text(
        yaml.safe_dump(
            {
                "model": "pib-cloud",
                "provider": "pib-cloud",
                "auxiliary": {"title_generation": {"model_upgrade_enabled": True}},
            }
        ),
        encoding="utf-8",
    )

    _ensure_mcp_servers_pib(str(pdir), model="pib-cloud", provider="pib-cloud")

    with open(pdir / "config.yaml", encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)
    assert cfg["auxiliary"]["title_generation"]["model_upgrade_enabled"] is False


def test_provisioning_writes_profile_config_version(tmp_path):
    """PR-2071: every profile rewrite stamps the current config_version."""
    from pib_hermes_config import PROFILE_CONFIG_VERSION, PROFILE_CONFIG_VERSION_KEY

    pdir = tmp_path / "versioned"
    pdir.mkdir()
    (pdir / "config.yaml").write_text(
        "model: gemini-3.8-flash\nprovider: gemini\n", encoding="utf-8"
    )

    _ensure_mcp_servers_pib(str(pdir), model="gemini-3.8-flash", provider="gemini")

    with open(pdir / "config.yaml", encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)
    assert cfg[PROFILE_CONFIG_VERSION_KEY] == PROFILE_CONFIG_VERSION


def test_stale_config_version_is_repaired_once(tmp_path):
    from pib_hermes_config import PROFILE_CONFIG_VERSION, PROFILE_CONFIG_VERSION_KEY
    from public_api_client.hermes_agent_client import (
        profile_config_version_is_current,
        read_profile_config_version,
    )

    pdir = tmp_path / "stale"
    pdir.mkdir()
    (pdir / "config.yaml").write_text(
        yaml.safe_dump(
            {
                "model": "gemini-3.8-flash",
                "provider": "gemini",
                PROFILE_CONFIG_VERSION_KEY: PROFILE_CONFIG_VERSION - 1,
                "tools": PROFILE_TOOL_SEARCH_CONFIG,
                "auxiliary": {
                    "title_generation": {"model_upgrade_enabled": False},
                },
            }
        ),
        encoding="utf-8",
    )

    assert not profile_config_version_is_current(str(pdir))
    _ensure_mcp_servers_pib(str(pdir), model="gemini-3.8-flash", provider="gemini")
    assert read_profile_config_version(str(pdir)) == PROFILE_CONFIG_VERSION

    with patch("yaml.safe_dump") as dump:
        _ensure_mcp_servers_pib(str(pdir), model="gemini-3.8-flash", provider="gemini")
        dump.assert_not_called()


def test_profile_config_check_reports_drift_and_exits_nonzero(tmp_path, capsys):
    from pib_hermes_config import PROFILE_CONFIG_VERSION_KEY, PROFILE_PREFIX
    from public_api_client.hermes_daemon import run_profile_config_check

    profiles_root = tmp_path / "profiles"
    profile_dir = profiles_root / f"{PROFILE_PREFIX}drift-me"
    profile_dir.mkdir(parents=True)
    (profile_dir / "config.yaml").write_text(
        "model: gemini-3.8-flash\nprovider: gemini\n", encoding="utf-8"
    )

    assert run_profile_config_check(str(profiles_root)) == 1
    err = capsys.readouterr().err
    assert PROFILE_CONFIG_VERSION_KEY in err
    assert "drift-me" in err

    _ensure_mcp_servers_pib(
        str(profile_dir), model="gemini-3.8-flash", provider="gemini"
    )
    assert run_profile_config_check(str(profiles_root)) == 0
