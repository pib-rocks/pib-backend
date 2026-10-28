"""PR-2102: warm-turn preparation — key cache, transcript seeding, repair safety net."""

from __future__ import annotations

import sys
from pathlib import Path
from unittest.mock import MagicMock

import contextlib

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
VOICE_ASSISTANT_PKG = REPO_ROOT / "ros_packages" / "voice_assistant"
if str(VOICE_ASSISTANT_PKG) not in sys.path:
    sys.path.insert(0, str(VOICE_ASSISTANT_PKG))

from pib_hermes_config.conversation_transcript import (  # noqa: E402
    flask_messages_for_session_seed,
    repair_message_sequence,
)
from public_api_client import hermes_agent_client as client  # noqa: E402
from public_api_client import hermes_daemon as hd  # noqa: E402
from test_hermes_daemon import _FakeSessionDB, _flask_message  # noqa: E402


class _FlaskRow:
    def __init__(self, message_id: str, content: str, *, is_user: bool):
        self.message_id = message_id
        self.content = content
        self.is_user = is_user


def _roles(messages):
    return [row.get("role") for row in messages if isinstance(row, dict)]


def test_provider_key_cached_until_key_epoch_changes(monkeypatch):
    fetches = {"n": 0}
    epochs = iter([4, 4, 5])

    monkeypatch.setattr(
        "pib_api_client.key_store_client.read_key_store_epoch",
        lambda: next(epochs),
    )

    def fake_resolve(provider, api_name=None):
        fetches["n"] += 1
        return f"secret-{fetches['n']}", "key_store"

    monkeypatch.setattr(
        "voice_assistant.direct_tool_loop.resolve_hermes_provider_key",
        fake_resolve,
    )
    monkeypatch.setattr(
        client,
        "profile_hermes_settings",
        lambda profile_dir=None: (
            "gemini-3.8-flash",
            "pib-cloud",
            "https://api.pib.rocks/v1",
            ("PIB_CLOUD_API_KEY",),
        ),
    )
    client.clear_provider_key_cache()

    assert client._cached_provider_key_for_current_profile() == "secret-1"
    assert client._cached_provider_key_for_current_profile() == "secret-1"
    assert client._cached_provider_key_for_current_profile() == "secret-2"
    assert fetches["n"] == 2


def test_flask_seed_excludes_current_user_line():
    rows = [
        _FlaskRow("a", "Hallo", is_user=True),
        _FlaskRow("b", "Hi", is_user=False),
        _FlaskRow("c", "Nochmal", is_user=True),
    ]
    seedable = flask_messages_for_session_seed(rows, "Nochmal")
    assert [row.message_id for row in seedable] == ["a", "b"]


def test_seeding_current_user_before_agent_creates_user_user_wedge():
    """Load-bearing: without excluding the current user, repair must fire."""
    store = _FakeSessionDB()
    session_id = "pib_chat_wedge"
    flask_rows = [
        _flask_message("a", "first", True),
        _flask_message("b", "reply", False),
        _flask_message("c", "second question", True),
    ]
    current = "second question"

    # Buggy seed: copy every flask row including the live user line.
    hd._seed_hermes_session_from_flask_messages(
        store,
        session_id,
        "chat-wedge",
        list(flask_rows),
        current_user_text=None,
    )
    store.append_turn(session_id, current, "answer two")
    transcript, _display = store.get_resume_conversations(session_id)
    assert repair_message_sequence(None, transcript) > 0


def test_full_turn_sequence_stored_transcript_needs_no_repair():
    store = _FakeSessionDB()
    session_id = "pib_chat_clean"
    chat_id = "chat-clean"

    def run_turn(user_text: str, assistant: str, prior_flask):
        flask_rows = list(prior_flask) + [
            _flask_message(f"u-{user_text}", user_text, True)
        ]
        hd._seed_hermes_session_from_flask_messages(
            store,
            session_id,
            chat_id,
            flask_rows,
            current_user_text=user_text,
        )
        store.append_turn(session_id, user_text, assistant)

    run_turn("Hallo", "Hi.", [])
    run_turn(
        "Nochmal",
        "Sure.",
        [_flask_message("a", "Hallo", True), _flask_message("b", "Hi.", False)],
    )
    run_turn(
        "Und?",
        "Ok.",
        [
            _flask_message("a", "Hallo", True),
            _flask_message("b", "Hi.", False),
            _flask_message("c", "Nochmal", True),
            _flask_message("d", "Sure.", False),
        ],
    )

    transcript, _display = store.get_resume_conversations(session_id)
    assert _roles(transcript) == [
        "user",
        "assistant",
        "user",
        "assistant",
        "user",
        "assistant",
    ]
    assert repair_message_sequence(None, transcript) == 0


def test_run_turn_in_process_skips_ensure_profile_when_complete(monkeypatch, tmp_path):
    personality_id = "warm-skip"
    profile_dir = tmp_path / f"pib_{personality_id}"
    profile_dir.mkdir()
    for name in hd.PROFILE_DIRS:
        (profile_dir / name).mkdir()
    (profile_dir / "config.yaml").write_text(
        "model: m\nprovider: p\n", encoding="utf-8"
    )
    (profile_dir / ".env").write_text("# stub\n", encoding="utf-8")
    (profile_dir / "SOUL.md").write_text("soul", encoding="utf-8")

    monkeypatch.setattr(
        "pib_hermes_config.profile_dir_for", lambda _pid: str(profile_dir)
    )

    ensure_calls = {"n": 0}
    real_ensure = client.ensure_profile

    def counting_ensure(*args, **kwargs):
        ensure_calls["n"] += 1
        return real_ensure(*args, **kwargs)

    monkeypatch.setattr(client, "ensure_profile", counting_ensure)
    monkeypatch.setattr(hd, "hermes_home_scope", lambda _p: contextlib.nullcontext())
    monkeypatch.setattr(hd, "in_process_available", lambda: False)
    monkeypatch.setattr(hd, "_run_turn_in_home", lambda **kwargs: "ok")
    monkeypatch.setattr(client, "provider_key_for_turn", lambda: "k")

    hd.run_turn_in_process("Hi", "chat-skip", personality_id=personality_id)
    assert ensure_calls["n"] == 0
