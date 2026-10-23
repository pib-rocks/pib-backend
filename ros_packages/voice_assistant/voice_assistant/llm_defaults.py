"""Default model parameters shared by every request path.

Thinking is off by default (PR-2039). Measured on the robot: an identical prompt
answered in 3.39 s with the model's default dynamic thinking (218 thought tokens)
and in 1.66 s with the budget at zero. In the direct tool loop every round pays
it again and the thought count grows with the context (309 -> 982 tokens over
four rounds), which dominated a 40-74 s turn.

Kept in one stdlib-only module so both the generateContent path and the Gemini
Live path read the same value and a unit test can pin it without importing rclpy,
google-genai or the ROS stubs.
"""

from __future__ import annotations

THINKING_BUDGET = 0


def gemini_thinking_config() -> dict[str, int]:
    """``generationConfig.thinkingConfig`` for generateContent requests."""
    return {"thinkingBudget": THINKING_BUDGET}


def live_thinking_config() -> dict[str, int]:
    """``thinking_config`` for the Gemini Live connect config."""
    return {"thinking_budget": THINKING_BUDGET}
