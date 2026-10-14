"""Periodic MEMORY.md consolidation for every Hermes profile.

The voice assistant is the process that has the Gemini key (password.env).
The API container does not. Summaries therefore run here, on the same
profiles directory the API reads when it reports memory size.
"""

from __future__ import annotations

import logging

from pib_hermes_config.memory import (
    CONSOLIDATION_CHECK_SECONDS,
    consolidate_due_profiles,
)

logger = logging.getLogger(__name__)

SUMMARY_INSTRUCTION = (
    "Summarise these older memory entries into one short stable paragraph. "
    "Keep only facts that are already in the entries. Do not add facts. "
    "Reply with the summary only."
)


def summarise_old_entries(old_text: str) -> str | None:
    """One Gemini summary, or None when no key or no usable answer is available."""
    from voice_assistant.direct_tool_loop import (
        DirectToolLoopError,
        gemini_api_key,
        gemini_complete,
    )

    if not gemini_api_key():
        return None
    try:
        result = gemini_complete(
            [
                {"role": "system", "content": SUMMARY_INSTRUCTION},
                {"role": "user", "content": old_text},
            ],
            None,
        )
    except DirectToolLoopError:
        logger.warning("memory consolidation summary was refused")
        return None
    text = result.get("text") if isinstance(result, dict) else None
    if not isinstance(text, str) or not text.strip():
        return None
    return text.strip()


def run_periodic_consolidation(now: float | None = None) -> dict[str, str]:
    """Consolidate every profile whose last pass is older than the interval."""
    return consolidate_due_profiles(summarise_old_entries, now=now)


def schedule_memory_consolidation(node) -> None:
    """Check once an hour, and once shortly after the node is up."""
    node.create_timer(CONSOLIDATION_CHECK_SECONDS, node.consolidate_memories)
    executor = getattr(node, "_hermes_executor", None)
    if executor is not None:
        executor.submit(node.consolidate_memories)
