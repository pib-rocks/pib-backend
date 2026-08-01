from public_api_client.hermes_agent_client import (
    FALLBACK_REPLY,
    build_default_soul_text,
    ensure_profile,
    session_name_for,
    build_command,
    hermes_binary_available,
    profile_name_for,
    run_turn,
    uses_hermes_backend,
)
from pib_hermes_config import MCP_TOOLS_SOUL_SECTION
import os
import subprocess
from unittest.mock import patch


EXPECTED_MCP_TOOLS = (
    "mcp__pib__list_motors",
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
    "mcp_pib_list_motors",
    "mcp_pib_get_state",
    "mcp_pib_list_poses",
    "mcp_pib_list_programs",
    "mcp_pib_capture_image",
    "mcp_pib_move_motor",
    "mcp_pib_apply_pose",
    "mcp_pib_run_program",
    "mcp_pib_set_led",
    "mcp_pib_set_relay",
    "mcp_pib_soul_append",
)


def test_session_name_is_prefixed_and_sanitized():
    assert session_name_for("abc-123") == "pib_chat_abc-123"


def test_session_name_strips_unsafe_chars():
    assert session_name_for("a/b c!") == "pib_chat_ab_c"


def test_uses_hermes_backend_for_hermes_api_name():
    assert uses_hermes_backend("hermes-agent") is True
    assert uses_hermes_backend("gpt-4o") is False


def test_profile_name_is_derived_from_personality():
    assert profile_name_for("abc-123") == "pib_abc-123"


def test_build_command_uses_oneshot_named_session_and_profile():
    cmd = build_command("hallo", "chat-1", personality_id="p-9")
    assert cmd[0].endswith("hermes")
    assert "-p" in cmd and "pib_p-9" in cmd          # profile carries the SOUL.md
    assert "-z" in cmd and "hallo" in cmd
    assert "--continue" in cmd and "pib_chat_chat-1" in cmd  # durable per-chat session


def test_build_command_without_personality_omits_profile():
    cmd = build_command("hallo", "chat-1", personality_id=None)
    assert "-p" not in cmd


def test_build_default_soul_text_starts_with_robot_identity():
    text = build_default_soul_text("Eva")
    assert text.startswith("Du bist der humanoide Roboter Eva.")


def test_build_default_soul_text_substitutes_personality_name():
    text = build_default_soul_text("Thomas")
    assert "Du bist der humanoide Roboter Thomas." in text
    assert "{personality_name}" not in text


def test_build_default_soul_text_includes_custom_description():
    text = build_default_soul_text("Eva", custom_description="Sei freundlich und neugierig.")
    assert text.startswith("Du bist der humanoide Roboter Eva.")
    assert "Sei freundlich und neugierig." in text


def test_build_default_soul_text_documents_mcp_tools():
    text = build_default_soul_text("pib")
    assert "## Verfügbare MCP-Werkzeuge (pib_mcp_server)" in text
    for tool in EXPECTED_MCP_TOOLS:
        assert tool in text
    # Section body stays in sync with the module constant.
    assert MCP_TOOLS_SOUL_SECTION.strip() in text


def test_build_default_soul_text_defaults_blank_name_to_pib():
    text = build_default_soul_text("  ")
    assert text.startswith("Du bist der humanoide Roboter pib.")


def test_ensure_profile_writes_templated_soul_with_name_and_mcp_docs(
    tmp_path, monkeypatch
):
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path / "profiles"))
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
    for tool in EXPECTED_MCP_TOOLS:
        assert tool in content


def test_ensure_profile_defaults_personality_name_to_pib(tmp_path, monkeypatch):
    monkeypatch.setenv("PIB_HERMES_PROFILES_DIR", str(tmp_path / "profiles"))
    monkeypatch.setenv("PIB_HERMES_BIN", str(tmp_path / "not-installed" / "hermes"))

    pdir = ensure_profile("p-9", soul_text="Hallo.")
    with open(os.path.join(pdir, "SOUL.md"), encoding="utf-8") as fh:
        content = fh.read()
    assert content.startswith("Du bist der humanoide Roboter pib.")
    assert "Hallo." in content


def test_run_turn_returns_stdout(installed_hermes_bin, monkeypatch):
    # Force subprocess path: no warm daemon on this port.
    monkeypatch.setenv("PIB_HERMES_DAEMON_URL", "http://127.0.0.1:1")
    completed = subprocess.CompletedProcess(args=[], returncode=0, stdout="Hallo!\n", stderr="")
    with patch("subprocess.run", return_value=completed):
        assert run_turn("hi", "c1") == "Hallo!"


def test_run_turn_on_timeout_returns_fallback(installed_hermes_bin, monkeypatch):
    monkeypatch.setenv("PIB_HERMES_DAEMON_URL", "http://127.0.0.1:1")
    with patch("subprocess.run", side_effect=subprocess.TimeoutExpired(cmd="x", timeout=1)):
        out = run_turn("hi", "c1")
        assert out  # non-empty graceful sentence
        assert "moment" in out.lower() or "später" in out.lower()


def test_run_turn_on_error_returns_fallback(installed_hermes_bin, monkeypatch):
    monkeypatch.setenv("PIB_HERMES_DAEMON_URL", "http://127.0.0.1:1")
    completed = subprocess.CompletedProcess(args=[], returncode=1, stdout="", stderr="boom")
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


def test_run_turn_prefers_daemon_reply_over_subprocess(installed_hermes_bin, monkeypatch):
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

    with patch(
        "requests.post", return_value=_Resp()
    ):
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
