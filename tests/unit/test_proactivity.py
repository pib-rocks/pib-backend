"""Proactivity tiers, speech budget, and school profile."""

from __future__ import annotations

import pytest

from pib_capability.manifest import load_manifest
from pib_hermes_config.proactivity import (
    APPROACH_SKILL,
    PROACTIVE_TOPIC_SKILL,
    TIER_ANNOUNCED,
    TIER_SILENT,
    UnknownProactivityTierError,
    ProactivityBudget,
    decide,
    tier_for,
    validate_proactivity,
)


def _manifest(**overrides: str) -> dict:
    doc = load_manifest("pib5museum")
    block = dict(doc["proactivity"])
    block["overrides"] = {**block["overrides"], **overrides}
    return {**doc, "proactivity": block}


def test_tier_lookup_uses_override_then_default():
    manifest = _manifest(move_motor="announced")
    assert tier_for(manifest, "move_motor") == TIER_ANNOUNCED
    assert tier_for(manifest, "capture_image") == TIER_SILENT


def test_unknown_tier_in_manifest_is_rejected():
    doc = load_manifest("pib5museum")
    doc = {
        **doc,
        "proactivity": {
            **doc["proactivity"],
            "overrides": {**doc["proactivity"]["overrides"], "move_motor": "loud"},
        },
    }
    with pytest.raises(UnknownProactivityTierError):
        validate_proactivity(doc)


def test_fourth_unprompted_utterance_in_ten_minutes_is_refused():
    budget = ProactivityBudget()
    manifest = _manifest()
    base = 1_000_000
    for offset in (0, 121_000, 242_000):
        decision = decide(manifest, "run_program", base + offset, budget=budget)
        assert decision.allowed, decision.reason
        assert decision.reason

    fourth = decide(manifest, "run_program", base + 365_000, budget=budget)
    assert not fourth.allowed
    assert fourth.reason
    assert "10" in fourth.reason or "600" in fourth.reason


def test_second_approach_to_same_person_within_five_minutes_is_refused():
    budget = ProactivityBudget()
    manifest = _manifest()
    now = 2_000_000
    first = decide(manifest, APPROACH_SKILL, now, "ada", budget=budget)
    assert first.allowed
    assert first.reason
    second = decide(manifest, APPROACH_SKILL, now + 60_000, "ada", budget=budget)
    assert not second.allowed
    assert second.reason
    assert second.approaches_last_5m == 1


def test_silent_skill_is_unaffected_by_speech_budget():
    budget = ProactivityBudget()
    manifest = _manifest()
    now = 3_000_000
    for _ in range(5):
        decision = decide(manifest, "capture_image", now, budget=budget)
        assert decision.allowed
        assert decision.tier == TIER_SILENT
        assert decision.reason


def test_school_profile_suppresses_proactive_topic_initiation():
    doc = load_manifest("pib5edu")
    assert doc["proactivity"]["profile"] == "school"
    budget = ProactivityBudget()
    decision = decide(doc, PROACTIVE_TOPIC_SKILL, 4_000_000, budget=budget)
    assert not decision.allowed
    assert "school" in decision.reason


def test_every_decision_carries_a_reason():
    budget = ProactivityBudget()
    manifest = _manifest()
    cases = [
        ("run_program", None),
        ("capture_image", None),
        (APPROACH_SKILL, "person-a"),
        (PROACTIVE_TOPIC_SKILL, None),
    ]
    for skill, person in cases:
        decision = decide(
            manifest, skill, 5_000_000, person, budget=ProactivityBudget()
        )
        assert decision.reason.strip()
