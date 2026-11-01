"""PR-2122: in-container voice health probe."""

from __future__ import annotations

import sys
from pathlib import Path
from unittest.mock import patch

import pytest

REPO = Path(__file__).resolve().parents[2]
VOICE_PKG = REPO / "ros_packages" / "voice_assistant"
if str(VOICE_PKG) not in sys.path:
    sys.path.insert(0, str(VOICE_PKG))

from voice_assistant import voice_healthcheck  # noqa: E402


def test_healthcheck_exits_nonzero_only_for_real_failures():
    with patch.object(
        voice_healthcheck, "check_ros_stack", return_value=(True, "", "")
    ):
        with patch.object(
            voice_healthcheck,
            "check_provider_key",
            return_value=(True, "", "", "gemini"),
        ):
            assert voice_healthcheck.main([]) == 0

    with patch.object(
        voice_healthcheck,
        "check_ros_stack",
        return_value=(False, voice_healthcheck.REASON_CHAT_NODE, "missing nodes"),
    ):
        assert voice_healthcheck.main([]) == 1

    with patch.object(
        voice_healthcheck, "check_ros_stack", return_value=(True, "", "")
    ):
        with patch.object(
            voice_healthcheck,
            "check_provider_key",
            return_value=(
                False,
                voice_healthcheck.REASON_NO_KEY,
                "No keys are available for provider gemini.",
                "gemini",
            ),
        ):
            assert voice_healthcheck.main([]) == 1


def test_healthcheck_ros_stack_requires_chat_nodes():
    """Load-bearing: a running assistant without /chat or /audio_recorder must fail the node check."""
    stdout = "/voice_assistant\n/other\n"

    def fake_run(fragment, timeout):
        if "node list" in fragment:
            return 0, stdout, ""
        return 0, "/send_chat_message\n", ""

    with patch.object(voice_healthcheck, "_run_ros_command", side_effect=fake_run):
        ok, reason, detail = voice_healthcheck.check_ros_stack()

    assert ok is False
    assert reason == voice_healthcheck.REASON_CHAT_NODE
    assert "/chat" in detail
    assert "/audio_recorder" in detail


def test_healthcheck_ros_stack_succeeds_when_all_three_nodes_present():
    stdout = "/voice_assistant\n/chat\n/audio_recorder\n"

    def fake_run(fragment, timeout):
        if "node list" in fragment:
            return 0, stdout, ""
        return 0, "/send_chat_message\n", ""

    with patch.object(voice_healthcheck, "_run_ros_command", side_effect=fake_run):
        ok, reason, detail = voice_healthcheck.check_ros_stack()

    assert ok is True
    assert reason == ""


def test_healthcheck_provider_key_surfaces_no_key_reason(monkeypatch):
    """Load-bearing: stderr reason matches the .112 failure mode for docker inspect."""
    from voice_assistant.direct_tool_loop import DirectToolLoopError

    with patch(
        "public_api_client.hermes_agent_client.profile_hermes_settings",
        return_value=("gemini-3-flash", "gemini", None, ("GEMINI_API_KEY",)),
    ):
        with patch(
            "public_api_client.hermes_agent_client.provider_key_for_turn",
            side_effect=DirectToolLoopError(
                "No keys are available for provider gemini."
            ),
        ):
            with patch(
                "pib_hermes_config.provider_needs_key",
                return_value=True,
            ):
                ok, reason, detail, provider = voice_healthcheck.check_provider_key()

    assert ok is False
    assert reason == voice_healthcheck.REASON_NO_KEY
    assert provider == "gemini"
    # A present key stays invisible when the route asks for another credential id.
    # The detail must name that id and the config file, otherwise it reads as a missing key.
    assert "credential=gemini-3-flash" in detail
    assert "config=" in detail

    with patch.object(
        voice_healthcheck, "check_ros_stack", return_value=(True, "", "")
    ):
        with patch.object(
            voice_healthcheck,
            "check_provider_key",
            return_value=(False, voice_healthcheck.REASON_NO_KEY, detail, provider),
        ):
            with patch("sys.stderr") as err:
                captured = []

                def _write(text):
                    captured.append(text)

                err.write = _write
                code = voice_healthcheck.main([])

    assert code == 1
    joined = "".join(captured)
    assert "reason=no_key" in joined
    assert "provider=gemini" in joined
