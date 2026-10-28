"""Per-personality chat channel: Smart (Hermes) or Direct.

The channel is independent of the provider. Smart is the default. When the
installer recorded ``--direct-llm``, Smart is absent: every turn is Direct
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


#: Appended after the robot body description. The model already receives every motor
#: range in that block; a direct turn for "open your left hand" still spent five
#: requests calling list_motors, get_motion_model, list_poses and get_state, each a
#: full model round of 2-10 s. The sentence tells it when a tool is actually needed
#: and warns against reading a state out of the sign of a number.
ROBOT_CONTEXT_INSTRUCTION = (
    "These ranges come from the robot's own motor configuration. Plan a movement with "
    "them and call get_motion_model or list_motors only when you need live positions or "
    "settings, not to look up a limit. A larger number is not 'more open': use the named "
    "state where the model gives one, and ask instead of guessing an unlabelled end."
)

#: Compact motion-model text appended to every personality system prompt (Smart and Direct).
ROBOT_CONTEXT_MAX_CHARS = 2000

#: Prepended when a turn should emit Blockly/Python (see voice_assistant.chat).
CODE_GENERATION_DESCRIPTION_PREFIX = ""


def _normalize_robot_context(robot_context: str | None) -> str:
    if not isinstance(robot_context, str) or not robot_context.strip():
        return ""
    body = robot_context.strip()
    if len(body) > ROBOT_CONTEXT_MAX_CHARS:
        body = body[:ROBOT_CONTEXT_MAX_CHARS].rstrip()
    return body


def _resolve_soul_text(
    *,
    soul_text: str | None = None,
    personality_name: str | None = None,
    custom_description: str | None = None,
) -> str:
    if isinstance(soul_text, str) and soul_text.strip():
        return soul_text.strip()
    if personality_name is not None or custom_description is not None:
        from pib_hermes_config import _soul_identity_text

        return _soul_identity_text(
            personality_name or "pib", custom_description
        ).rstrip("\n")
    from pib_hermes_config import DEFAULT_SOUL

    return DEFAULT_SOUL


def _assemble_system_prompt(
    identity: str,
    *,
    robot_context: str | None = None,
    code_generation: bool = False,
    images_shown: bool = False,
    trailing_newline: bool = False,
) -> str:
    parts = [identity]
    if images_shown:
        parts.append(CAPTURED_IMAGE_SENTENCE)
    body = _normalize_robot_context(robot_context)
    if body:
        parts.extend([ROBOT_CONTEXT_INSTRUCTION, body])
    prompt = "\n\n".join(parts)
    if code_generation and CODE_GENERATION_DESCRIPTION_PREFIX:
        prompt = CODE_GENERATION_DESCRIPTION_PREFIX + prompt
    if trailing_newline and not prompt.endswith("\n"):
        prompt += "\n"
    return prompt


def personality_system_prompt(
    *,
    soul_text: str | None = None,
    personality_name: str | None = None,
    custom_description: str | None = None,
    robot_context: str | None = None,
    code_generation: bool = False,
    trailing_newline: bool = False,
) -> str:
    """Render the complete system prompt for one personality.

    Smart stores the result in ``SOUL.md``; Direct uses the same text and may add
    transport-only sentences (captured images) via ``direct_system_prompt``.
    """
    identity = _resolve_soul_text(
        soul_text=soul_text,
        personality_name=personality_name,
        custom_description=custom_description,
    )
    return _assemble_system_prompt(
        identity,
        robot_context=robot_context,
        code_generation=code_generation,
        trailing_newline=trailing_newline,
    )


def direct_system_prompt(
    soul_text: str | None,
    *,
    images_shown: bool = False,
    robot_context: str | None = None,
    code_generation: bool = False,
) -> str:
    """The personality's one identity text, used as the Direct system prompt.

    MEMORY.md is a Hermes file. This function does not read it.

    ``images_shown`` adds what the chat does with a captured picture: the model
    cannot look at the chat, so it is told rather than asked to guess.

    ``robot_context`` appends the robot body description from the motion model API.
    """
    identity = _resolve_soul_text(soul_text=soul_text)
    return _assemble_system_prompt(
        identity,
        robot_context=robot_context,
        code_generation=code_generation,
        images_shown=images_shown,
    )
