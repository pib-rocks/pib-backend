"""The voice assistant runs the periodic pass, and only with a real summary."""

import sys
from pathlib import Path
from unittest.mock import MagicMock

REPO_ROOT = Path(__file__).resolve().parents[2]
VOICE_ASSISTANT_PKG = REPO_ROOT / "ros_packages" / "voice_assistant"
if str(VOICE_ASSISTANT_PKG) not in sys.path:
    sys.path.insert(0, str(VOICE_ASSISTANT_PKG))

from pib_hermes_config.memory import CONSOLIDATION_CHECK_SECONDS  # noqa: E402
from voice_assistant import direct_tool_loop as direct_tool_loop_module  # noqa: E402
from voice_assistant.memory_consolidation import (  # noqa: E402
    schedule_memory_consolidation,
    summarise_old_entries,
)


def test_missing_gemini_key_does_not_invent_a_summary(monkeypatch):
    monkeypatch.delenv("GEMINI_API_KEY", raising=False)
    monkeypatch.delenv("GOOGLE_API_KEY", raising=False)
    assert summarise_old_entries("The garden is on the left.") is None


def test_summary_uses_the_pinned_gemini_call(monkeypatch):
    monkeypatch.setenv("GEMINI_API_KEY", "test-key")
    captured = {}

    def complete(messages, tools):
        captured["messages"] = messages
        captured["tools"] = tools
        return {"text": " Garden left. "}

    monkeypatch.setattr(
        "voice_assistant.direct_tool_loop.gemini_complete",
        complete,
    )
    assert summarise_old_entries("The garden is on the left.") == "Garden left."
    assert captured["tools"] is None
    assert captured["messages"][1]["content"] == "The garden is on the left."


def test_schedule_checks_once_an_hour_and_once_at_startup():
    node = MagicMock()
    schedule_memory_consolidation(node)
    node.create_timer.assert_called_once_with(
        CONSOLIDATION_CHECK_SECONDS, node.consolidate_memories
    )
    node._hermes_executor.submit.assert_called_once_with(node.consolidate_memories)
