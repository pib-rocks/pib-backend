"""Unit tests for per-turn episodic writes (PR-2228)."""

from __future__ import annotations

import logging

import pytest

import pib_memory.turn_episode as turn_episode
from pib_memory.store import MemoryStore
from pib_memory.turn_episode import record_turn


@pytest.fixture
def memory_db(tmp_path, monkeypatch):
    path = tmp_path / "memory.db"
    monkeypatch.setenv("PIB_MEMORY_DB_PATH", str(path))
    turn_episode._recorded.clear()
    yield path
    turn_episode._recorded.clear()


@pytest.fixture
def memory_store(memory_db):
    store = MemoryStore(memory_db)
    yield store
    store.close()


def test_record_turn_writes_one_episode(memory_store):
    record_turn(
        chat_id="chat-1",
        turn_id="abc12345",
        task="What time is it?",
        outcome="It is three o'clock.",
    )
    exported = memory_store.export()
    assert len(exported["episodes"]) == 1
    episode = exported["episodes"][0]
    assert episode["evidence_ref"] == "chat-1#abc12345"
    assert episode["task"] == "What time is it?"
    assert episode["outcome"] == "It is three o'clock."
    assert episode["failure_reason"] is None


def test_record_turn_truncates_task_and_outcome(memory_store):
    long_text = "x" * 300
    record_turn(
        chat_id="c",
        turn_id="t1",
        task=long_text,
        outcome=long_text,
    )
    episode = memory_store.export()["episodes"][0]
    assert len(episode["task"]) == 200
    assert len(episode["outcome"]) == 200


def test_record_turn_idempotent(memory_store):
    kwargs = dict(
        chat_id="chat-2",
        turn_id="turn-a",
        task="Hello",
        outcome="Hi there.",
    )
    record_turn(**kwargs)
    record_turn(**kwargs)
    assert len(memory_store.export()["episodes"]) == 1


def test_record_turn_skips_empty_task(memory_store):
    record_turn(chat_id="c", turn_id="t", task="", outcome="answer")
    record_turn(chat_id="c", turn_id="t", task="   ", outcome="answer")
    assert memory_store.export()["episodes"] == []


def test_record_turn_skips_missing_outcome(memory_store):
    record_turn(chat_id="c", turn_id="t", task="question", outcome="")
    record_turn(chat_id="c", turn_id="t", task="question", outcome=None)
    assert memory_store.export()["episodes"] == []


def test_record_turn_skips_missing_ids(memory_store):
    record_turn(chat_id="", turn_id="t", task="q", outcome="a")
    record_turn(chat_id="c", turn_id="", task="q", outcome="a")
    assert memory_store.export()["episodes"] == []


def test_record_turn_failure_reason(memory_store):
    record_turn(
        chat_id="c",
        turn_id="t",
        task="Do the thing",
        failure_reason="provider timeout",
    )
    episode = memory_store.export()["episodes"][0]
    assert episode["failure_reason"] == "provider timeout"
    assert episode["outcome"] is None


def test_record_turn_survives_unwritable_store(monkeypatch, caplog):
    turn_episode._recorded.clear()

    def broken_default():
        raise OSError("disk full")

    monkeypatch.setattr(
        turn_episode.MemoryStore, "default", staticmethod(broken_default)
    )
    with caplog.at_level(logging.WARNING):
        record_turn(
            chat_id="c",
            turn_id="t",
            task="hello",
            outcome="world",
        )
    assert "turn episode was not recorded" in caplog.text


def test_record_turn_recallable(memory_store):
    record_turn(
        chat_id="lab",
        turn_id="deadbeef",
        task="pick cup",
        outcome="done",
    )
    from pib_memory.recall import recall

    records = recall("cup", k=3, store=memory_store)
    assert records
    assert records[0]["evidence_ref"] == "lab#deadbeef"
