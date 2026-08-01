from public_api_client.hermes_agent_client import (
    FALLBACK_REPLY,
    session_name_for,
    build_command,
    hermes_binary_available,
    profile_name_for,
    run_turn,
    uses_hermes_backend,
)
import subprocess
from unittest.mock import patch


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


def test_run_turn_returns_stdout(installed_hermes_bin):
    completed = subprocess.CompletedProcess(args=[], returncode=0, stdout="Hallo!\n", stderr="")
    with patch("subprocess.run", return_value=completed):
        assert run_turn("hi", "c1") == "Hallo!"


def test_run_turn_on_timeout_returns_fallback(installed_hermes_bin):
    with patch("subprocess.run", side_effect=subprocess.TimeoutExpired(cmd="x", timeout=1)):
        out = run_turn("hi", "c1")
        assert out  # non-empty graceful sentence
        assert "moment" in out.lower() or "später" in out.lower()


def test_run_turn_on_error_returns_fallback(installed_hermes_bin):
    completed = subprocess.CompletedProcess(args=[], returncode=1, stdout="", stderr="boom")
    with patch("subprocess.run", return_value=completed):
        assert run_turn("hi", "c1")


def test_run_turn_without_installed_binary_returns_fallback(tmp_path, monkeypatch):
    monkeypatch.setenv("PIB_HERMES_BIN", str(tmp_path / "not-installed" / "hermes"))

    with patch("public_api_client.hermes_agent_client.subprocess.run") as run:
        out = run_turn("hi", "c1")

    assert out == FALLBACK_REPLY
    run.assert_not_called()  # never attempt a subprocess we know cannot start


def test_run_turn_treats_a_non_executable_binary_as_missing(tmp_path, monkeypatch):
    binary = tmp_path / "hermes"
    binary.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
    binary.chmod(0o644)
    monkeypatch.setenv("PIB_HERMES_BIN", str(binary))

    with patch("public_api_client.hermes_agent_client.subprocess.run") as run:
        assert run_turn("hi", "c1") == FALLBACK_REPLY

    run.assert_not_called()


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
    import importlib
    import public_api_client.hermes_agent_client as hac

    importlib.reload(hac)
    try:
        assert hac.DEFAULT_TIMEOUT_SECONDS == 77
    finally:
        monkeypatch.delenv("PIB_HERMES_TIMEOUT", raising=False)
        importlib.reload(hac)
