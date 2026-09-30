"""What a chat may do before the operator password is entered.

The named mode is ``degraded``. Local speech does not read the key store:
faster-whisper and Supertone need no provider key, and the Blockly speech
block calls the same local player. Smart and Direct chats do not run, and
they answer with one sentence that names the missing capability. The
assistant speaks that sentence with the personality's gender and language.
"""

from __future__ import annotations

import json
import os
from typing import Optional
from urllib.request import urlopen

MODE_DEGRADED = "degraded"
MODE_UNLOCKED = "unlocked"
MISSING_CAPABILITY = "operator password"
LOCAL_VOICE_IN = "faster-whisper"
LOCAL_VOICE_OUT = "supertone"

_STATUS_PATH = "/system/key-store"


def allows_cloud_chat(mode: str) -> bool:
    """Smart and Direct both need the unlocked store."""
    return mode == MODE_UNLOCKED


def local_voice_available(mode: str) -> bool:
    """Local speech in and out stay available in every mode, including degraded."""
    return mode in (MODE_DEGRADED, MODE_UNLOCKED)


def refusal_sentence(channel: str) -> str:
    """One sentence, in the first person, naming the channel and the missing capability."""
    if channel == "smart":
        label = "Smart"
    elif channel == "direct":
        label = "Direct"
    else:
        raise ValueError(f"unknown chat channel: {channel}")
    return (
        f"I can't open a {label} chat because the {MISSING_CAPABILITY} is missing, "
        "so the provider keys are still locked."
    )


def spoken_reply(channel: str, gender: str, language: str) -> dict[str, str]:
    """The refusal as the personality would say it, on the local Supertone voice."""
    return {
        "text": refusal_sentence(channel),
        "gender": gender,
        "language": language,
        "engine": LOCAL_VOICE_OUT,
    }


def fetch_operating_mode(
    base_url: Optional[str] = None,
    opener=None,
    timeout: float = 2.0,
) -> str:
    """Read the key-store mode. Anything unreadable is degraded, not a crash.

    A missing password and a key store this process cannot see are the same
    situation for a chat: the provider keys are not available, and the turn
    must not fall through to a cloud model.
    """
    if opener is None:
        opener = urlopen
    base = (
        base_url
        if base_url is not None
        else os.environ.get("FLASK_API_BASE_URL", "http://127.0.0.1:5000")
    ).rstrip("/")
    try:
        with opener(base + _STATUS_PATH, timeout=timeout) as response:
            payload = response.read().decode("utf-8")
        body = json.loads(payload)
    except Exception:
        return MODE_DEGRADED
    if isinstance(body, dict) and body.get("mode") == MODE_UNLOCKED:
        return MODE_UNLOCKED
    return MODE_DEGRADED
