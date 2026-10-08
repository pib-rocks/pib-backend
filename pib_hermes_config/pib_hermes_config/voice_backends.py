"""Per-personality speech backends.

The local engines are not provider rows. A cloud choice is a provider id,
and only a row whose capability flag is set may be stored. ElevenLabs is
not a backend. In live mode the provider's voice is used, so the
personality's local voice does not apply.
"""

from __future__ import annotations

LOCAL_STT_ID = "local_whisper"
LOCAL_TTS_ID = "supertone"
LOCAL_STT_ENGINE = "faster-whisper"
LOCAL_TTS_ENGINE = "supertone"

#: The recorder's previous cloud name. It is not offered: offering reads the
#: stt capability, not a list of provider names.
LEGACY_CLOUD_STT = "tryb_api"

LIVE_VOICE_NOTE = (
    "In live mode the provider's voice is used, "
    "so the personality's local voice does not apply."
)

_LOCAL_STT = frozenset({LOCAL_STT_ID, LOCAL_STT_ENGINE})
_LOCAL_TTS = frozenset({LOCAL_TTS_ID, LOCAL_TTS_ENGINE})


def local_voice_applies(live: bool) -> bool:
    """Gender and language feed the local engine only outside live mode."""
    return not live


def live_voice_note(live: bool) -> str | None:
    """Sentence the UI states while live mode is the selected path."""
    if live:
        return LIVE_VOICE_NOTE
    return None


def _reject_elevenlabs(value: str) -> None:
    if "elevenlabs" in value.lower():
        raise ValueError("ElevenLabs is out of scope.")


def normalize_stt_choice(value: object, provider_has_stt) -> str:
    """Store the local engine, or the id of a provider with stt."""
    if not isinstance(value, str) or not value.strip():
        return LOCAL_STT_ID
    raw = value.strip()
    _reject_elevenlabs(raw)
    if raw in _LOCAL_STT:
        return LOCAL_STT_ID
    if raw.isdigit() and provider_has_stt(int(raw)):
        return str(int(raw))
    raise ValueError(
        "Speech-to-text must be the local faster-whisper engine "
        "or a provider with the stt capability."
    )


def normalize_tts_choice(value: object, provider_has_tts) -> str:
    """Store the local engine, or the id of a provider with tts."""
    if not isinstance(value, str) or not value.strip():
        return LOCAL_TTS_ID
    raw = value.strip()
    _reject_elevenlabs(raw)
    if raw in _LOCAL_TTS:
        return LOCAL_TTS_ID
    if raw.isdigit() and provider_has_tts(int(raw)):
        return str(int(raw))
    raise ValueError(
        "Text-to-speech must be the local Supertone engine "
        "or a provider with the tts capability."
    )


def resolve_stt_route(engine: object) -> str:
    """``local``, ``tryb`` (legacy cloud client), or ``unsupported``.

    ``unsupported`` has no speech client in this repo. Callers must not
    send that audio to a different provider.
    """
    if not isinstance(engine, str) or engine.strip() in ("", *_LOCAL_STT):
        return "local"
    if engine.strip() == LEGACY_CLOUD_STT:
        return "tryb"
    return "unsupported"


def resolve_tts_route(engine: object) -> str:
    """``local`` or ``unsupported``. The local engine is Supertone."""
    if not isinstance(engine, str) or engine.strip() in ("", *_LOCAL_TTS):
        return "local"
    return "unsupported"
