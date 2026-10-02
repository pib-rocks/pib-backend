"""Live-session rules shared by the registry and the voice node.

The live model identifier is a field of the provider row. The ``live``
capability flag gates a session. Context-window compression stays on, an
unused session ends at the personality's idle timeout, and one personality
holds the voice channel at a time.
"""

from __future__ import annotations

#: September 2025 preview. It must not be opened again.
RETIRED_LIVE_MODEL = "gemini-2.5-flash-native-audio-preview-09-2025"

#: Confirmed on this account's Gemini ``/v1beta/models`` list on the date below
#: (``models/gemini-3.1-flash-live-preview``). Preview ids change, so a later
#: unlock re-reads the account list and rewrites the row.
GEMINI_LIVE_MODEL = "gemini-3.1-flash-live-preview"
GEMINI_LIVE_MODEL_CHECKED_ON = "2026-09-30"

#: OpenAI realtime model. It is stored only after ``/v1/models`` on that
#: account lists it. No OpenAI key was available to check on 2026-09-30.
OPENAI_LIVE_MODEL = "gpt-realtime"

VOICE_MODE_LIVE = "live"
VOICE_MODE_TURN_BASED = "turn_based"
VOICE_MODES = (VOICE_MODE_LIVE, VOICE_MODE_TURN_BASED)

#: D18 requires a per-personality idle timeout and does not name a duration.
#: Sixty seconds is the stored default so an unused live session stops.
DEFAULT_LIVE_IDLE_TIMEOUT_SECONDS = 60

GEMINI_MODELS_URL = "https://generativelanguage.googleapis.com/v1beta/models"


def live_candidate_for(api_name: str) -> str | None:
    """The live model this provider row may pin, or None when it has none."""
    name = (api_name or "").lower()
    if "gemini" in name:
        return GEMINI_LIVE_MODEL
    if name.startswith("gpt-"):
        return OPENAI_LIVE_MODEL
    return None


def gemini_live_connect_model(model: object) -> str | None:
    """Model id the existing Gemini live client may open.

    The retired preview is refused. ``gpt-realtime`` is not a Gemini model;
    this process has no OpenAI realtime transport for it.
    """
    if not isinstance(model, str):
        return None
    chosen = model.strip()
    if not chosen or chosen == RETIRED_LIVE_MODEL:
        return None
    if "gemini" not in chosen.lower():
        return None
    return chosen


def voice_start_mode(voice_mode: object, live_capable: bool, live_model: object) -> str:
    """The mode the voice button is about to start.

    Live only when the personality asks for it, the provider's live flag is
    set, and the pinned model is one this process can open.
    """
    if voice_mode != VOICE_MODE_LIVE or not live_capable:
        return VOICE_MODE_TURN_BASED
    if gemini_live_connect_model(live_model) is None:
        return VOICE_MODE_TURN_BASED
    return VOICE_MODE_LIVE


def normalize_voice_mode(value: object) -> str:
    if value is None or value == "":
        return VOICE_MODE_LIVE
    if value in VOICE_MODES:
        return str(value)
    raise ValueError("Voice mode must be live or turn_based.")


def normalize_idle_timeout(value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError("Live idle timeout must be a whole number of seconds.")
    if value < 1:
        raise ValueError("Live idle timeout must be at least 1 second.")
    return value


def context_window_compression(trigger_tokens: int, target_tokens: int) -> dict:
    """Always-on compression. It is what removes the 15-minute audio cap."""
    return {
        "trigger_tokens": int(trigger_tokens),
        "sliding_window": {"target_tokens": int(target_tokens)},
    }


def idle_expired(last_activity: float, now: float, timeout_s: float) -> bool:
    """True when a live session has had no speech for ``timeout_s`` seconds."""
    if timeout_s <= 0:
        return False
    return (now - last_activity) >= timeout_s


def channel_turn_on_allowed(
    channel_on: bool,
    holder_personality_id: object,
    requested_personality_id: object,
) -> bool:
    """Another personality cannot take an open voice channel.

    Releasing the channel (turning it off) is always allowed. A missing holder
    does not lock the channel.
    """
    if not channel_on:
        return True
    if not holder_personality_id or not requested_personality_id:
        return True
    return holder_personality_id == requested_personality_id


def live_session_ends_on_handover(
    channel_on: bool,
    holder_chat_id: object,
    requested_chat_id: object,
    live_running: bool,
) -> bool:
    """A live session ends when the voice moves to another chat.

    It does not end merely because some other personality asked and was refused.
    """
    if not live_running or not channel_on:
        return False
    if not holder_chat_id or not requested_chat_id:
        return False
    return holder_chat_id != requested_chat_id


def openai_models_url(endpoint_base: object) -> str:
    """``/v1/models`` on the OpenAI API or an OpenAI-compatible base."""
    if not isinstance(endpoint_base, str) or not endpoint_base.strip():
        return "https://api.openai.com/v1/models"
    base = endpoint_base.strip().rstrip("/")
    if base.endswith("/v1"):
        return base + "/models"
    return base + "/v1/models"
