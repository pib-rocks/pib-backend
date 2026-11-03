"""Unit tests for robot-local episodic memory (PR-2212)."""

from __future__ import annotations

import json

import pytest

from pib_memory.recall import recall
from pib_memory.store import MemoryStore, MissingEvidenceRefError


@pytest.fixture
def memory_store(tmp_path):
    store = MemoryStore(tmp_path / "memory.db")
    yield store
    store.close()


def test_record_types_round_trip(memory_store):
    ts = 1_700_000_000_000
    ep_id = memory_store.write_episode(
        ts_ms=ts,
        place_id="lab-table",
        participants=["Ada"],
        task="pick cup",
        outcome="success",
        failure_reason=None,
        evidence_ref="frame:abc123",
        consent_scope="household",
        created_ms=ts,
    )
    obj_id = memory_store.upsert_object(
        object_id="cup-1",
        label="red cup",
        place_id="lab-table",
        last_seen_ms=ts,
        confidence=0.92,
        frame_ref="frame:cup-1",
    )
    person_id = memory_store.upsert_person(
        display_name="Ada",
        embedding_ref="embed:ada",
        consent="yes",
        preferences={"drink": "tea"},
        created_ms=ts,
    )

    exported = memory_store.export()
    episode = next(e for e in exported["episodes"] if e["id"] == ep_id)
    assert episode["ts_ms"] == ts
    assert episode["place_id"] == "lab-table"
    assert json.loads(episode["participants_json"]) == ["Ada"]
    assert episode["task"] == "pick cup"
    assert episode["outcome"] == "success"
    assert episode["failure_reason"] is None
    assert episode["evidence_ref"] == "frame:abc123"
    assert episode["consent_scope"] == "household"
    assert episode["created_ms"] == ts

    obj = next(o for o in exported["object_memory"] if o["id"] == obj_id)
    assert obj["object_id"] == "cup-1"
    assert obj["label"] == "red cup"
    assert obj["place_id"] == "lab-table"
    assert obj["last_seen_ms"] == ts
    assert obj["confidence"] == pytest.approx(0.92)
    assert obj["frame_ref"] == "frame:cup-1"

    person = next(p for p in exported["person_memory"] if p["id"] == person_id)
    assert person["display_name"] == "Ada"
    assert person["embedding_ref"] == "embed:ada"
    assert person["consent"] == "yes"
    assert json.loads(person["preferences_json"]) == {"drink": "tea"}
    assert person["created_ms"] == ts


def test_write_episode_requires_evidence_ref(memory_store):
    with pytest.raises(MissingEvidenceRefError):
        memory_store.write_episode(evidence_ref="")
    with pytest.raises(MissingEvidenceRefError):
        memory_store.write_episode(evidence_ref="   ")


def test_delete_person_removes_person_and_episodes(memory_store):
    memory_store.upsert_person(display_name="Bob", embedding_ref="embed:bob")
    memory_store.write_episode(
        participants=["Bob"],
        task="wave",
        outcome="done",
        evidence_ref="frame:wave",
    )
    memory_store.write_episode(
        participants=["Ada"],
        task="nod",
        outcome="done",
        evidence_ref="frame:nod",
    )

    memory_store.delete_person("Bob")

    exported = memory_store.export()
    assert not any(p["display_name"] == "Bob" for p in exported["person_memory"])
    remaining_participants = [
        json.loads(ep["participants_json"]) for ep in exported["episodes"]
    ]
    assert all("Bob" not in parts for parts in remaining_participants)
    assert any("Ada" in parts for parts in remaining_participants)


def test_recall_respects_k_order_and_fields(memory_store):
    base = 1_700_000_000_000
    memory_store.write_episode(
        ts_ms=base,
        task="older task alpha",
        outcome="ok",
        evidence_ref="frame:old",
    )
    memory_store.write_episode(
        ts_ms=base + 60_000,
        task="newer task alpha",
        outcome="ok",
        evidence_ref="frame:new",
    )
    memory_store.write_episode(
        ts_ms=base + 120_000,
        task="unrelated beta",
        outcome="ok",
        evidence_ref="frame:beta",
    )

    records = recall("alpha", k=2, store=memory_store)
    assert len(records) <= 2
    assert all("date" in r and "evidence_ref" in r and "confidence" in r for r in records)
    assert records[0]["evidence_ref"] == "frame:new"
    assert records[1]["evidence_ref"] == "frame:old"


def test_purge_removes_only_older_rows(memory_store):
    cutoff = 1_700_000_000_000
    memory_store.write_episode(
        ts_ms=cutoff - 10_000,
        task="ancient",
        evidence_ref="frame:ancient",
        created_ms=cutoff - 10_000,
    )
    memory_store.write_episode(
        ts_ms=cutoff + 10_000,
        task="recent",
        evidence_ref="frame:recent",
        created_ms=cutoff + 10_000,
    )
    memory_store.upsert_object(
        object_id="o-old",
        last_seen_ms=cutoff - 1,
        frame_ref="frame:o-old",
    )
    memory_store.upsert_object(
        object_id="o-new",
        last_seen_ms=cutoff + 1,
        frame_ref="frame:o-new",
    )
    memory_store.upsert_person(
        display_name="OldPat",
        embedding_ref="embed:old",
        created_ms=cutoff - 1,
    )
    memory_store.upsert_person(
        display_name="NewPat",
        embedding_ref="embed:new",
        created_ms=cutoff + 1,
    )

    memory_store.purge(cutoff)

    exported = memory_store.export()
    assert {ep["evidence_ref"] for ep in exported["episodes"]} == {"frame:recent"}
    assert {o["object_id"] for o in exported["object_memory"]} == {"o-new"}
    assert {p["display_name"] for p in exported["person_memory"]} == {"NewPat"}
