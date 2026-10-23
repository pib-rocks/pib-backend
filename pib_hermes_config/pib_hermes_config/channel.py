"""Per-personality chat channel: Smart (Hermes) or Direct.

The channel is independent of the provider. Smart is the default. When the
installer recorded ``--no-smart-chats``, Smart is absent: every turn is Direct
and the stored channel is left untouched, so removing the marker restores it.

``PIB_SMART_CHATS`` overrides the marker file. An empty value means "read the
file". A missing file means Smart is available.
"""

from __future__ import annotations

import os

CHANNEL_SMART = "smart"
CHANNEL_DIRECT = "direct"
CHANNELS = (CHANNEL_SMART, CHANNEL_DIRECT)

SMART_CHATS_ENV = "PIB_SMART_CHATS"
SMART_CHATS_FILE = "/etc/pib_smart_chats"

_DISABLED = frozenset({"0", "false", "no", "disabled", "off"})


def _as_enabled(raw: str) -> bool:
    return raw.strip().lower() not in _DISABLED


def smart_chats_enabled() -> bool:
    """False only when the installer (or an explicit override) disabled Hermes."""
    raw = os.environ.get(SMART_CHATS_ENV)
    if raw is not None and raw.strip() != "":
        return _as_enabled(raw)
    if not os.path.isfile(SMART_CHATS_FILE):
        return True
    try:
        text = open(SMART_CHATS_FILE, encoding="utf-8").read()
    except OSError:
        return True
    if not text.strip():
        return True
    return _as_enabled(text)


def normalize_channel(value: object) -> str:
    """Stored channel. Anything other than the string Direct is Smart."""
    if isinstance(value, str) and value == CHANNEL_DIRECT:
        return CHANNEL_DIRECT
    return CHANNEL_SMART


def effective_channel(stored: object) -> str:
    """Channel a turn actually uses. The installer flag wins over the row."""
    if not smart_chats_enabled():
        return CHANNEL_DIRECT
    return normalize_channel(stored)


def turn_channel(personality: object) -> str:
    """Resolve the channel for one turn from a personality payload.

    ``effective_channel`` is the API's already-applied value. A local installer
    flag still forces Direct, so a stale client cannot keep a Smart turn.
    """
    reported = getattr(personality, "effective_channel", None)
    if isinstance(reported, str) and reported in CHANNELS:
        chosen: object = reported
    else:
        chosen = getattr(personality, "channel", None)
    return effective_channel(chosen)


#: A picture a tool captured is shown as an attachment under the answer (PC-119).
#: The chat surface is the one thing a model cannot see, so without this sentence
#: it answers that the interface is text-only and offers no picture at all - the
#: measured answer of a personality whose tool had just captured one.
CAPTURED_IMAGE_SENTENCE = (
    "A picture you capture with your camera is attached to your answer in the chat "
    "automatically. Do not claim that the chat is text-only or that it cannot show "
    "pictures, and do not ask for the picture again: describe the frame you just "
    "captured."
)


def direct_system_prompt(
    soul_text: str | None,
    *,
    images_shown: bool = False,
    robot_context: str | None = None,
) -> str:
    """The personality's one identity text, used as the Direct system prompt.

    MEMORY.md is a Hermes file. This function does not read it.

    ``images_shown`` adds what the chat does with a captured picture: the model
    cannot look at the chat, so it is told rather than asked to guess.

    ``robot_context`` appends the robot body description from the motion model API.
    """
    if isinstance(soul_text, str) and soul_text.strip():
        prompt = soul_text.strip()
    else:
        from pib_hermes_config import DEFAULT_SOUL

        prompt = DEFAULT_SOUL
    if images_shown:
        prompt = f"{prompt}\n\n{CAPTURED_IMAGE_SENTENCE}"
    if isinstance(robot_context, str) and robot_context.strip():
        prompt = f"{prompt}\n\n{robot_context.strip()}"
    return prompt
