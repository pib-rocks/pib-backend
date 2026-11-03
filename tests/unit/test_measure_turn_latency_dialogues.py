"""Dialogue corpus for tools/measure_turn_latency.py (PR-2207)."""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
TOOLS = REPO_ROOT / "tools"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

from measure_turn_latency import DIALOGUES, build_dialogue_plan  # noqa: E402


def test_dialogue_plan_has_twenty_dialogues_with_one_cancel_each():
    plan = build_dialogue_plan()
    assert plan is DIALOGUES
    assert len(plan) == 20
    for index, dialogue in enumerate(plan):
        assert 2 <= len(dialogue) <= 4, index
        cancel_turns = [turn for turn in dialogue if turn[1] is not None]
        assert len(cancel_turns) == 1, index
        delay_ms = cancel_turns[0][1]
        assert isinstance(delay_ms, int) and delay_ms > 0
