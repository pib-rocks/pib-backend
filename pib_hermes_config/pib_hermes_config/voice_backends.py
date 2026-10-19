"""Per-personality speech backends.

The local engines are not provider rows. Cloud whisper and cloud speech on
pib.Cloud are stored as their own ids, ``cloud_whisper`` and ``cloud_tts``.
Any other cloud choice is a provider id. Either is stored only while the
row's capability flag is set. ElevenLabs is not a backend. In live mode the
provider's voice is used, so the personality's local voice does not apply.
"""

from __future__ import annotations

import os

LOCAL_STT_ID = "local_whisper"
LOCAL_TTS_ID = "supertone"
LOCAL_STT_ENGINE = "faster-whisper"
LOCAL_TTS_ENGINE = "supertone"

#: Cloud whisper on pib.Cloud. It is not the pib.Cloud row id, so the stored
#: value names the engine and its route is never the legacy tryb_api client.
CLOUD_STT_ID = "cloud_whisper"
CLOUD_STT_ROUTE = "cloud"

#: Cloud speech on pib.Cloud. Same shape as cloud whisper: the stored value
#: names the engine, never the bare row id, and its route is ``cloud``.
CLOUD_TTS_ID = "cloud_tts"
CLOUD_TTS_ROUTE = "cloud"

#: Language the local STT engine is pinned to when nothing overrides it.
#: faster-whisper's own default (``language=None``) runs a language-ID pass
#: over 99 languages on every turn; measured on the test robot that pass cost
#: about half the turn (37.0 s -> 19.2 s for one 8 s recording, PC-39), and on
#: silence the detector reported 'en' with p=0.34.
DEFAULT_STT_LANGUAGE = "de"
STT_LANGUAGE_ENV = "STT_LANGUAGE"
#: Value that asks for the engine's own detection again.
STT_LANGUAGE_AUTO = "auto"

#: Spellings the rest of the code base uses for the same language.
_STT_LANGUAGE_CODES = {
    "de": "de",
    "deu": "de",
    "ger": "de",
    "german": "de",
    "deutsch": "de",
    "en": "en",
    "eng": "en",
    "english": "en",
    "englisch": "en",
}

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


def normalize_stt_choice(
    value: object,
    provider_has_stt,
    cloud_stt_model_id: int | None = None,
    default: str = LOCAL_STT_ID,
) -> str:
    """Store the local engine, cloud whisper, or the id of a provider with stt.

    ``cloud_stt_model_id`` is the pib.Cloud row id while that row carries the
    stt capability, otherwise None. Cloud whisper is accepted only then, and
    the bare pib.Cloud row id is stored as ``cloud_whisper``. A missing or
    blank value is no choice and stores ``default``.
    """
    if not isinstance(value, str) or not value.strip():
        return default
    raw = value.strip()
    _reject_elevenlabs(raw)
    if raw in _LOCAL_STT:
        return LOCAL_STT_ID
    names_cloud = raw == CLOUD_STT_ID or (
        raw.isdigit()
        and cloud_stt_model_id is not None
        and int(raw) == cloud_stt_model_id
    )
    if names_cloud:
        if cloud_stt_model_id is None:
            raise ValueError("Cloud whisper needs pib.Cloud with the stt capability.")
        return CLOUD_STT_ID
    if raw.isdigit() and provider_has_stt(int(raw)):
        return str(int(raw))
    raise ValueError(
        "Speech-to-text must be the local faster-whisper engine "
        "or a provider with the stt capability."
    )


def normalize_tts_choice(
    value: object,
    provider_has_tts,
    cloud_tts_model_id: int | None = None,
    default: str = LOCAL_TTS_ID,
) -> str:
    """Store the local engine, cloud speech, or the id of a provider with tts.

    ``cloud_tts_model_id`` is the pib.Cloud row id while that row carries the
    tts capability, otherwise None. Cloud speech is accepted only then, and
    the bare pib.Cloud row id is stored as ``cloud_tts``. A missing or blank
    value is no choice and stores ``default``.
    """
    if not isinstance(value, str) or not value.strip():
        return default
    raw = value.strip()
    _reject_elevenlabs(raw)
    if raw in _LOCAL_TTS:
        return LOCAL_TTS_ID
    names_cloud = raw == CLOUD_TTS_ID or (
        raw.isdigit()
        and cloud_tts_model_id is not None
        and int(raw) == cloud_tts_model_id
    )
    if names_cloud:
        if cloud_tts_model_id is None:
            raise ValueError("Cloud speech needs pib.Cloud with the tts capability.")
        return CLOUD_TTS_ID
    if raw.isdigit() and provider_has_tts(int(raw)):
        return str(int(raw))
    raise ValueError(
        "Text-to-speech must be the local Supertone engine "
        "or a provider with the tts capability."
    )


def resolve_stt_route(engine: object) -> str:
    """``local``, ``cloud`` (pib.Cloud whisper), ``tryb`` (legacy cloud
    client), or ``unsupported``.

    ``cloud`` is not ``tryb``: cloud whisper must not be sent to the legacy
    client. ``unsupported`` has no speech client in this repo. Callers must
    not send that audio to a different provider.
    """
    if not isinstance(engine, str) or engine.strip() in ("", *_LOCAL_STT):
        return "local"
    if engine.strip() == CLOUD_STT_ID:
        return CLOUD_STT_ROUTE
    if engine.strip() == LEGACY_CLOUD_STT:
        return "tryb"
    return "unsupported"


def resolve_tts_route(engine: object) -> str:
    """``local``, ``cloud`` (pib.Cloud speech), or ``unsupported``.

    The local engine is Supertone. ``cloud`` is pib.Cloud's own speech
    endpoint; ``unsupported`` has no speech client in this repo.
    """
    if not isinstance(engine, str) or engine.strip() in ("", *_LOCAL_TTS):
        return "local"
    if engine.strip() == CLOUD_TTS_ID:
        return CLOUD_TTS_ROUTE
    return "unsupported"


def whisper_language_code(value: object) -> str:
    """ISO 639-1 code for faster-whisper, or "" when it cannot be pinned.

    Accepts both the code (``de``, ``de-DE``) and the spelled-out names the
    rest of the code base carries (``German``, ``Deutsch``). Anything unknown
    returns "", which leaves the engine on its own detection instead of
    guessing a language that would silently mistranslate a turn.
    """
    if not isinstance(value, str):
        return ""
    raw = value.strip()
    if not raw:
        return ""
    lowered = raw.lower()
    if lowered in _STT_LANGUAGE_CODES:
        return _STT_LANGUAGE_CODES[lowered]
    code = lowered.replace("_", "-").split("-", 1)[0]
    if len(code) == 2 and code.isalpha():
        return code
    return ""


def configured_stt_language() -> str:
    """Language the local engine transcribes in, as a code ("" = detect).

    Ships pinned to German. ``STT_LANGUAGE`` overrides it; ``auto`` asks for
    the engine's own detection again.
    """
    raw = os.getenv(STT_LANGUAGE_ENV)
    if raw is None or not raw.strip():
        return DEFAULT_STT_LANGUAGE
    if raw.strip().lower() == STT_LANGUAGE_AUTO:
        return ""
    return whisper_language_code(raw)
