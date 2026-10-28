"""PR-2068: flask chat messages are the conversation record for Smart and Direct."""

from __future__ import annotations

import sys
import types
from unittest.mock import MagicMock, patch

import pytest

from public_api_client import hermes_daemon as hd
from test_hermes_daemon import (
    _FakeSessionDB,
    _fake_agent_module,
    _flask_message,
)


@pytest.fixture(autouse=True)
def isolate_mcp_discovery(monkeypatch):
    monkeypatch.setattr(hd, "_mcp_discovery_attempted", set())
    monkeypatch.setattr(hd, "_discover_mcp_tools", MagicMock())
    monkeypatch.setattr(hd, "_registered_mcp_tool_count", MagicMock(return_value=0))
    monkeypatch.setattr(hd, "_session_dbs", {})
    monkeypatch.setattr(hd, "_session_db_attempted", set())
    monkeypatch.setattr(hd, "_create_session_db", MagicMock(return_value=None))


@pytest.fixture(autouse=True)
def matching_hermes_venv(monkeypatch):
    monkeypatch.setattr(hd, "in_process_available", lambda: True)


def test_smart_turn_replays_direct_messages_from_flask(monkeypatch):
    """After Direct turns, Smart must see the same flask history."""
    store = _FakeSessionDB()
    monkeypatch.setattr(hd, "_create_session_db", MagicMock(return_value=store))
    created = []
    module = _fake_agent_module(created, store_turns=True, reply="Smart.")
    hd.clear_agent_cache()

    flask_rows = [
        _flask_message("m3", "Was war das Codewort?", True),
        _flask_message("m2", "Notiert.", False),
        _flask_message("m1", "Codewort BLAU.", True),
    ]

    with (
        patch.dict(sys.modules, {"run_agent": module}),
        patch.object(
            hd,
            "_fetch_flask_chat_history",
            return_value=(True, flask_rows),
        ),
    ):
        hd.run_turn_in_process(
            "Was war das Codewort?",
            "chat-switch",
            message_history=5,
        )

    agent = created[0]
    assert agent.histories[0] == [
        {"role": "user", "content": "Codewort BLAU."},
        {"role": "assistant", "content": "Notiert."},
    ]
    assert len(store.batch_appends) == 1
    seeded_ids = [
        row["display_metadata"][hd.PIB_FLASK_MESSAGE_ID_KEY]
        for row in store.batch_appends[0][1]
    ]
    assert seeded_ids == ["m1", "m2", "m3"]


def test_seeding_flask_messages_twice_does_not_duplicate(monkeypatch):
    store = _FakeSessionDB()
    monkeypatch.setattr(hd, "_create_session_db", MagicMock(return_value=store))
    created = []
    module = _fake_agent_module(created)
    hd.clear_agent_cache()

    flask_rows = [
        _flask_message("m3", "Nochmal", True),
        _flask_message("m2", "Hi", False),
        _flask_message("m1", "Hallo", True),
    ]

    with (
        patch.dict(sys.modules, {"run_agent": module}),
        patch.object(
            hd,
            "_fetch_flask_chat_history",
            return_value=(True, flask_rows),
        ),
    ):
        hd.run_turn_in_process("Nochmal", "chat-dup", message_history=5)
        hd.run_turn_in_process("Und?", "chat-dup", message_history=5)

    assert len(store.batch_appends) == 1
    assert len(store.transcripts["pib_chat_chat-dup"]) == 3


def test_history_length_follows_personality_message_history(monkeypatch):
    store = _FakeSessionDB()
    monkeypatch.setattr(hd, "_create_session_db", MagicMock(return_value=store))
    created = []
    module = _fake_agent_module(created)
    hd.clear_agent_cache()

    flask_rows = [
        _flask_message("m7", "msg-7", True),
        _flask_message("m6", "msg-6", False),
        _flask_message("m5", "msg-5", True),
    ]

    with (
        patch.dict(sys.modules, {"run_agent": module}),
        patch.object(
            hd,
            "_fetch_flask_chat_history",
            return_value=(True, flask_rows),
        ) as fetch,
    ):
        hd.run_turn_in_process("latest", "chat-window", message_history=3)

    fetch.assert_called_once_with("chat-window", 3)
    history = created[0].histories[0]
    assert history == [
        {"role": "user", "content": "msg-5"},
        {"role": "assistant", "content": "msg-6"},
        {"role": "user", "content": "msg-7"},
    ]


def test_unknown_chat_history_fetch_returns_answer_not_exception(monkeypatch):
    store = _FakeSessionDB()
    monkeypatch.setattr(hd, "_create_session_db", MagicMock(return_value=store))
    created = []
    module = _fake_agent_module(created)
    hd.clear_agent_cache()

    with (
        patch.dict(sys.modules, {"run_agent": module}),
        patch.object(
            hd,
            "_fetch_flask_chat_history",
            return_value=(False, None),
        ),
    ):
        reply = hd.run_turn_in_process(
            "Hallo",
            "unknown-chat-2068",
            message_history=5,
        )

    assert reply == "OK."
    assert created[0].histories[0] is None
    assert store.batch_appends == []


def test_flask_messages_missing_for_seed_helpers():
    flask_rows = [
        _flask_message("a", "one", True),
        _flask_message("b", "two", False),
    ]
    session_rows = [
        {
            "role": "user",
            "content": "one",
            "display_metadata": {hd.PIB_FLASK_MESSAGE_ID_KEY: "a"},
        },
        {"role": "assistant", "content": "two"},
    ]
    missing = hd._flask_messages_missing_for_seed(flask_rows, session_rows)
    assert [message.message_id for message in missing] == []

    session_rows[1]["display_metadata"] = {}
    missing = hd._flask_messages_missing_for_seed(flask_rows, session_rows)
    assert [message.message_id for message in missing] == []
