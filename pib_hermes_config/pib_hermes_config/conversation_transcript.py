"""Hermes session transcript helpers for the pib smart-turn path.

The warm daemon seeds Flask chat rows into ``hermes_state.SessionDB`` before
``AIAgent.run_conversation`` persists the live turn. Seeding the current user
line and letting the agent append it again leaves a durable ``user;user`` wedge;
``get_resume_conversations`` then runs ``repair_message_sequence`` on every
turn. These helpers keep the stored transcript alternation-clean at write time.
"""

from __future__ import annotations

from typing import Any, Optional


def _plain_user_content(content: object) -> bool:
    return isinstance(content, str)


def _merge_consecutive_users(messages: list[dict]) -> tuple[list[dict], int]:
    repairs = 0
    merged: list[dict] = []
    for message in messages:
        if not isinstance(message, dict):
            merged.append(message)
            continue
        previous = merged[-1] if merged and isinstance(merged[-1], dict) else None
        if (
            previous is not None
            and previous.get("role") == "user"
            and message.get("role") == "user"
            and _plain_user_content(previous.get("content"))
            and _plain_user_content(message.get("content"))
        ):
            prev_text = str(previous.get("content") or "")
            new_text = str(message.get("content") or "")
            previous["content"] = (
                f"{prev_text}\n\n{new_text}"
                if prev_text and new_text
                else (prev_text or new_text)
            )
            repairs += 1
            continue
        merged.append(message)
    return merged, repairs


def _merge_consecutive_assistants(messages: list[dict]) -> tuple[list[dict], int]:
    repairs = 0
    merged: list[dict] = []
    for message in messages:
        if not isinstance(message, dict):
            merged.append(message)
            continue
        previous = merged[-1] if merged and isinstance(merged[-1], dict) else None
        if (
            previous is not None
            and previous.get("role") == "assistant"
            and message.get("role") == "assistant"
            and not previous.get("tool_calls")
            and not message.get("tool_calls")
        ):
            prev_text = str(previous.get("content") or "")
            new_text = str(message.get("content") or "")
            previous["content"] = (
                f"{prev_text}\n\n{new_text}"
                if prev_text and new_text
                else (prev_text or new_text)
            )
            repairs += 1
            continue
        merged.append(message)
    return merged, repairs


def _drop_leading_tool_rows(messages: list[dict]) -> tuple[list[dict], int]:
    index = 0
    while index < len(messages):
        role = (
            messages[index].get("role") if isinstance(messages[index], dict) else None
        )
        if role != "tool":
            break
        index += 1
    if index == 0:
        return messages, 0
    return messages[index:], index


def repair_message_sequence(_agent: object, messages: list[dict]) -> int:
    """Collapse role-alternation violations in place; returns the repair count.

    Subset of Hermes' ``agent.agent_runtime_helpers.repair_message_sequence`` —
    enough for the pib seed/turn path and unit tests. The SessionDB restore path
    in Hermes keeps the full implementation as the safety net.
    """
    if not messages:
        return 0
    repairs = 0
    current = list(messages)
    for repair_pass in (
        _merge_consecutive_assistants,
        _drop_leading_tool_rows,
        _merge_consecutive_users,
    ):
        current, made = repair_pass(current)
        repairs += made
    if repairs:
        messages[:] = current
    return repairs


def flask_messages_for_session_seed(
    flask_messages: list[Any],
    current_user_text: Optional[str] = None,
) -> list[Any]:
    """Flask rows safe to batch-append before ``run_conversation`` runs.

    The chat backend persists the spoken user line before the Hermes turn starts.
    That row must not be copied into the session store ahead of the agent, or the
    agent's own user append creates consecutive user rows.
    """
    if not flask_messages:
        return []
    if not isinstance(current_user_text, str) or not current_user_text.strip():
        return list(flask_messages)
    last = flask_messages[-1]
    if not getattr(last, "is_user", False):
        return list(flask_messages)
    if getattr(last, "content", None) != current_user_text:
        return list(flask_messages)
    return list(flask_messages[:-1])


def simulate_smart_turn_transcript(
    store,
    session_id: str,
    flask_messages: list[Any],
    current_user_text: str,
    assistant_reply: str,
    *,
    seed_flask_rows,
    append_turn,
) -> list[dict]:
    """One smart turn: seed flask history, then persist the agent reply pair."""
    seedable = flask_messages_for_session_seed(flask_messages, current_user_text)
    missing = seed_flask_rows(seedable, session_id)
    if missing:
        batch = [
            {
                "role": "user" if message.is_user else "assistant",
                "content": message.content,
            }
            for message in missing
        ]
        store.append_messages_batch(session_id, batch)
    append_turn(session_id, current_user_text, assistant_reply)
    restored, _display = store.get_resume_conversations(session_id)
    return list(restored or [])
