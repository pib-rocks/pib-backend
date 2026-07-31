"""Runs the Hermes Agent as the conversation partner for a pib chat.

One pib chat_id maps to exactly one persistent Hermes session, so the agent
retains memory across turns and across robot restarts.
"""
import logging
import os
import re
import subprocess
from typing import Optional

HERMES_BIN = os.environ.get("PIB_HERMES_BIN", "/home/pib/.local/bin/hermes")
SESSION_PREFIX = "pib_chat_"
PROFILE_PREFIX = "pib_"
DEFAULT_TIMEOUT_SECONDS = int(os.environ.get("PIB_HERMES_TIMEOUT", "120"))

_UNSAFE = re.compile(r"[^A-Za-z0-9_-]")


def session_name_for(chat_id: str) -> str:
    """Deterministic Hermes session name for a pib chat id."""
    return SESSION_PREFIX + _UNSAFE.sub("", (chat_id or "").replace(" ", "_"))


def profile_name_for(personality_id: str) -> str:
    """Hermes profile that holds this personality's SOUL.md."""
    return PROFILE_PREFIX + _UNSAFE.sub("", (personality_id or "").replace(" ", "_"))


def build_command(
    text: str,
    chat_id: str,
    personality_id: Optional[str] = None,
    toolsets: Optional[str] = None,
) -> list[str]:
    """argv for one one-shot turn in this chat's persistent session.

    The personality's persona comes from the Hermes PROFILE
    (~/.hermes/profiles/pib_<personality_id>/SOUL.md), selected via -p.
    Conversation memory comes from the named SESSION, selected via -c.
    Verified: -p and -c compose correctly (persona + memory together).
    """
    cmd = [HERMES_BIN]
    if personality_id:
        cmd += ["-p", profile_name_for(personality_id)]
    cmd += ["-z", text, "-c", session_name_for(chat_id)]
    if toolsets:
        cmd += ["-t", toolsets]
    return cmd
