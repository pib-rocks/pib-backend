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
DEFAULT_TIMEOUT_SECONDS = int(os.environ.get("PIB_HERMES_TIMEOUT", "120"))

_UNSAFE = re.compile(r"[^A-Za-z0-9_-]")


def session_name_for(chat_id: str) -> str:
    """Deterministic Hermes session name for a pib chat id."""
    return SESSION_PREFIX + _UNSAFE.sub("", (chat_id or "").replace(" ", "_"))
