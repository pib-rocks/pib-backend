"""Startup password prompt on the robot's own display.

The host browser opens the prompt URL. The display node, inside the
container, only reads the key-store mode. Neither call is allowed to take
the display down: an unreachable key store is a retry, not an error.
"""

from __future__ import annotations

import json
import os
from typing import Optional
from urllib.request import urlopen

MODE_DEGRADED = "degraded"
MODE_UNLOCKED = "unlocked"
MAX_ATTEMPTS = 15

_STATUS_PATH = "/system/key-store"
_PROMPT_PATH = "/system/key-store/display"


def prompt_decision(mode: Optional[str]) -> str:
    """``open`` the page, ``skip`` it, or ``retry`` the read."""
    if mode == MODE_DEGRADED:
        return "open"
    if mode == MODE_UNLOCKED:
        return "skip"
    return "retry"


def password_prompt_url() -> str:
    """URL the host browser loads. Chromium runs on the host, not in the container."""
    configured = os.environ.get("PIB_KEY_STORE_PROMPT_URL")
    if configured:
        return configured
    return "http://127.0.0.1:5000" + _PROMPT_PATH


def status_url() -> str:
    base = os.environ.get("FLASK_API_BASE_URL", "http://127.0.0.1:5000").rstrip("/")
    return base + _STATUS_PATH


def read_operating_mode(opener=None, timeout: float = 1.5) -> Optional[str]:
    """Return ``degraded`` or ``unlocked``, or None when the store cannot be read."""
    if opener is None:
        opener = urlopen
    try:
        with opener(status_url(), timeout=timeout) as response:
            body = json.loads(response.read().decode("utf-8"))
    except Exception:
        return None
    if not isinstance(body, dict):
        return None
    mode = body.get("mode")
    if mode in (MODE_DEGRADED, MODE_UNLOCKED):
        return mode
    return None
