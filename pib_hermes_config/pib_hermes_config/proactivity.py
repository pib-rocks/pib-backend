"""Four-tier proactivity from the capability manifest and unprompted speech budget."""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Any, Mapping

TIER_SILENT = "silent"
TIER_SIGNALLED = "signalled"
TIER_ANNOUNCED = "announced"
TIER_FORBIDDEN = "forbidden"

VALID_TIERS = frozenset({TIER_SILENT, TIER_SIGNALLED, TIER_ANNOUNCED, TIER_FORBIDDEN})

PROFILE_DEFAULT = "default"
PROFILE_SCHOOL = "school"
VALID_PROFILES = frozenset({PROFILE_DEFAULT, PROFILE_SCHOOL})

PRESENCE_GREETING_SKILL = "presence_greeting"
PROACTIVE_TOPIC_SKILL = "proactive_topic"
APPROACH_SKILL = "approach"

UTTERANCE_WINDOW_SHORT_MS = 120_000
UTTERANCE_MAX_SHORT = 1
UTTERANCE_WINDOW_LONG_MS = 600_000
UTTERANCE_MAX_LONG = 3
APPROACH_WINDOW_MS = 300_000
APPROACH_MAX_PER_PERSON = 1

# Skills that are not MCP tools but still carry a tier when declared in overrides.
ATTENTION_SKILLS = frozenset(
    {PRESENCE_GREETING_SKILL, PROACTIVE_TOPIC_SKILL, APPROACH_SKILL}
)


class UnknownProactivityTierError(ValueError):
    """A manifest declares a proactivity tier this build does not recognise."""

    def __init__(self, tier: str, *, context: str) -> None:
        self.tier = tier
        self.context = context
        super().__init__(
            f"unknown proactivity tier {tier!r} in {context}; "
            f"expected one of: {', '.join(sorted(VALID_TIERS))}"
        )


def _validate_tier(tier: str, *, context: str) -> str:
    if tier not in VALID_TIERS:
        raise UnknownProactivityTierError(tier, context=context)
    return tier


def proactivity_block(manifest: Mapping[str, Any]) -> dict[str, Any]:
    block = manifest.get("proactivity")
    if not isinstance(block, dict):
        raise ValueError("manifest missing required key 'proactivity'")
    return block


def resolve_profile(manifest: Mapping[str, Any]) -> str:
    from_env = (os.getenv("PIB_PROACTIVITY_PROFILE") or "").strip()
    if from_env:
        if from_env not in VALID_PROFILES:
            raise ValueError(
                f"unknown PIB_PROACTIVITY_PROFILE {from_env!r}; "
                f"expected one of: {', '.join(sorted(VALID_PROFILES))}"
            )
        return from_env
    block = proactivity_block(manifest)
    profile = block.get("profile", PROFILE_DEFAULT)
    if not isinstance(profile, str) or profile not in VALID_PROFILES:
        raise ValueError(
            f"manifest proactivity.profile must be one of "
            f"{', '.join(sorted(VALID_PROFILES))}, got {profile!r}"
        )
    return profile


def tier_for(manifest: Mapping[str, Any], skill: str) -> str:
    block = proactivity_block(manifest)
    default = block.get("default")
    if not isinstance(default, str):
        raise ValueError("manifest proactivity.default must be a string")
    _validate_tier(default, context="proactivity.default")
    overrides = block.get("overrides")
    if overrides is None:
        overrides = {}
    if not isinstance(overrides, dict):
        raise ValueError("manifest proactivity.overrides must be an object")
    if skill in overrides:
        tier = overrides[skill]
        if not isinstance(tier, str):
            raise ValueError(
                f"manifest proactivity.overrides[{skill!r}] must be a string"
            )
        return _validate_tier(tier, context=f"proactivity.overrides[{skill!r}]")
    if skill == PRESENCE_GREETING_SKILL:
        return TIER_ANNOUNCED
    if skill == PROACTIVE_TOPIC_SKILL:
        return TIER_ANNOUNCED
    return default


def validate_proactivity(doc: Mapping[str, Any]) -> None:
    block = proactivity_block(doc)
    _validate_tier(block["default"], context="proactivity.default")
    overrides = block.get("overrides")
    if overrides is None:
        raise ValueError("manifest proactivity.overrides must be present")
    if not isinstance(overrides, dict):
        raise ValueError("manifest proactivity.overrides must be an object")
    for key, tier in overrides.items():
        if not isinstance(key, str) or not key.strip():
            raise ValueError(
                "manifest proactivity override keys must be non-empty strings"
            )
        if not isinstance(tier, str):
            raise ValueError(
                f"manifest proactivity.overrides[{key!r}] must be a string"
            )
        _validate_tier(tier, context=f"proactivity.overrides[{key!r}]")
    profile = block.get("profile", PROFILE_DEFAULT)
    if not isinstance(profile, str) or profile not in VALID_PROFILES:
        raise ValueError(
            f"manifest proactivity.profile must be one of "
            f"{', '.join(sorted(VALID_PROFILES))}, got {profile!r}"
        )


@dataclass
class ProactivityDecision:
    allowed: bool
    tier: str
    reason: str
    utterances_last_2m: int
    utterances_last_10m: int
    approaches_last_5m: int | None = None


class ProactivityBudget:
    """Sliding-window limits on unprompted speech and approach attempts."""

    def __init__(self) -> None:
        self._utterance_ms: list[int] = []
        self._approach_ms: dict[str, list[int]] = {}

    def _prune_utterances(self, now_ms: int) -> None:
        cutoff = now_ms - UTTERANCE_WINDOW_LONG_MS
        self._utterance_ms = [stamp for stamp in self._utterance_ms if stamp > cutoff]

    def utterance_counts(self, now_ms: int) -> tuple[int, int]:
        self._prune_utterances(now_ms)
        short_cutoff = now_ms - UTTERANCE_WINDOW_SHORT_MS
        short = sum(1 for stamp in self._utterance_ms if stamp > short_cutoff)
        long = len(self._utterance_ms)
        return short, long

    def allow_utterance(self, now_ms: int) -> bool:
        short, long = self.utterance_counts(now_ms)
        if short >= UTTERANCE_MAX_SHORT:
            return False
        if long >= UTTERANCE_MAX_LONG:
            return False
        return True

    def record_utterance(self, now_ms: int) -> None:
        self._prune_utterances(now_ms)
        self._utterance_ms.append(now_ms)

    def _prune_approaches(self, person_id: str, now_ms: int) -> None:
        cutoff = now_ms - APPROACH_WINDOW_MS
        stamps = self._approach_ms.get(person_id, [])
        stamps = [stamp for stamp in stamps if stamp > cutoff]
        if stamps:
            self._approach_ms[person_id] = stamps
        elif person_id in self._approach_ms:
            del self._approach_ms[person_id]

    def approach_count(self, now_ms: int, person_id: str) -> int:
        self._prune_approaches(person_id, now_ms)
        return len(self._approach_ms.get(person_id, []))

    def allow_approach(self, now_ms: int, person_id: str) -> bool:
        return self.approach_count(now_ms, person_id) < APPROACH_MAX_PER_PERSON

    def record_approach(self, now_ms: int, person_id: str) -> None:
        self._prune_approaches(person_id, now_ms)
        self._approach_ms.setdefault(person_id, []).append(now_ms)


def decide(
    manifest: Mapping[str, Any],
    skill: str,
    now_ms: int,
    person_id: str | None = None,
    *,
    budget: ProactivityBudget,
) -> ProactivityDecision:
    tier = tier_for(manifest, skill)
    short, long = budget.utterance_counts(now_ms)
    approaches: int | None = None

    if tier == TIER_FORBIDDEN:
        return ProactivityDecision(
            allowed=False,
            tier=tier,
            reason=f"skill {skill!r} is forbidden by manifest",
            utterances_last_2m=short,
            utterances_last_10m=long,
            approaches_last_5m=approaches,
        )

    profile = resolve_profile(manifest)
    if profile == PROFILE_SCHOOL and skill == PROACTIVE_TOPIC_SKILL:
        return ProactivityDecision(
            allowed=False,
            tier=tier,
            reason="school profile suppresses proactive topic initiation",
            utterances_last_2m=short,
            utterances_last_10m=long,
            approaches_last_5m=approaches,
        )

    if skill == APPROACH_SKILL:
        if person_id is None:
            return ProactivityDecision(
                allowed=False,
                tier=tier,
                reason="approach requires a person_id",
                utterances_last_2m=short,
                utterances_last_10m=long,
                approaches_last_5m=None,
            )
        approaches = budget.approach_count(now_ms, person_id)
        if not budget.allow_approach(now_ms, person_id):
            return ProactivityDecision(
                allowed=False,
                tier=tier,
                reason=(
                    f"approach budget exhausted for person {person_id!r} "
                    f"({approaches} in {APPROACH_WINDOW_MS // 1000}s window)"
                ),
                utterances_last_2m=short,
                utterances_last_10m=long,
                approaches_last_5m=approaches,
            )
        budget.record_approach(now_ms, person_id)
        return ProactivityDecision(
            allowed=True,
            tier=tier,
            reason="approach allowed",
            utterances_last_2m=short,
            utterances_last_10m=long,
            approaches_last_5m=approaches + 1,
        )

    if profile == PROFILE_SCHOOL and skill == PRESENCE_GREETING_SKILL:
        return ProactivityDecision(
            allowed=True,
            tier=TIER_SILENT,
            reason="school profile greets without speech",
            utterances_last_2m=short,
            utterances_last_10m=long,
            approaches_last_5m=approaches,
        )

    if tier == TIER_SILENT:
        return ProactivityDecision(
            allowed=True,
            tier=tier,
            reason="silent tier does not consume unprompted speech budget",
            utterances_last_2m=short,
            utterances_last_10m=long,
            approaches_last_5m=approaches,
        )

    if tier in (TIER_SIGNALLED, TIER_ANNOUNCED):
        if not budget.allow_utterance(now_ms):
            if short >= UTTERANCE_MAX_SHORT:
                reason = (
                    f"unprompted speech budget: {short} utterance(s) in "
                    f"{UTTERANCE_WINDOW_SHORT_MS // 1000}s (max {UTTERANCE_MAX_SHORT})"
                )
            else:
                reason = (
                    f"unprompted speech budget: {long} utterance(s) in "
                    f"{UTTERANCE_WINDOW_LONG_MS // 1000}s (max {UTTERANCE_MAX_LONG})"
                )
            return ProactivityDecision(
                allowed=False,
                tier=tier,
                reason=reason,
                utterances_last_2m=short,
                utterances_last_10m=long,
                approaches_last_5m=approaches,
            )
        budget.record_utterance(now_ms)
        short, long = budget.utterance_counts(now_ms)
        return ProactivityDecision(
            allowed=True,
            tier=tier,
            reason=f"{tier} tier allowed under speech budget",
            utterances_last_2m=short,
            utterances_last_10m=long,
            approaches_last_5m=approaches,
        )

    return ProactivityDecision(
        allowed=False,
        tier=tier,
        reason=f"unhandled tier {tier!r}",
        utterances_last_2m=short,
        utterances_last_10m=long,
        approaches_last_5m=approaches,
    )
