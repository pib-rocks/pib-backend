from datetime import date
from typing import List, Optional

from model.assistant_model import AssistantModel
from model.provider_model import Provider
from pib_hermes_config.live_session import (
    GEMINI_LIVE_MODEL,
    GEMINI_LIVE_MODEL_CHECKED_ON,
)
from pib_hermes_config.voice_backends import (
    LIVE_VOICE_NOTE,
    LOCAL_STT_ENGINE,
    LOCAL_STT_ID,
    LOCAL_TTS_ENGINE,
    LOCAL_TTS_ID,
)
from provider_registry import (
    capabilities_for,
    has_capability,
    has_images_capability,
    is_registry_default,
    pins_gemini_live_model,
)


def build_provider(model: AssistantModel) -> Provider:
    """One registry row for an assistant model, using that model's id.

    A Gemini chat id the catalogue marks live carries the live model pinned
    on 2026-09-30. Other rows stay unpinned until their own account list is read.
    """
    live_model = None
    checked_on = None
    if pins_gemini_live_model(model.api_name):
        live_model = GEMINI_LIVE_MODEL
        checked_on = date.fromisoformat(GEMINI_LIVE_MODEL_CHECKED_ON)
    return Provider(
        id=model.id,
        api_name=model.api_name,
        visual_name=model.visual_name,
        has_image_support=bool(model.has_image_support),
        endpoint_base=None,
        capabilities=capabilities_for(model.api_name, bool(model.has_image_support)),
        credential_ref=None,
        is_default=is_registry_default(model.api_name),
        live_model=live_model,
        live_model_checked_on=checked_on,
    )


def get_provider_by_id(provider_id: int) -> Optional[Provider]:
    return Provider.query.filter_by(id=provider_id).first()


def get_default_provider() -> Provider:
    return Provider.query.filter_by(is_default=True).one()


def _local_speech_option(option_id: str, engine: str, label: str) -> dict:
    return {
        "id": option_id,
        "kind": "local",
        "engine": engine,
        "label": label,
    }


def _provider_speech_option(row: Provider) -> dict:
    return {
        "id": str(row.id),
        "kind": "provider",
        "engine": row.api_name,
        "label": row.visual_name,
    }


def speech_backends() -> dict:
    """Local engines plus provider rows that carry stt or tts.

    The image filter does not apply here. A row is offered for speech only
    when its own capability flag is set. ElevenLabs is not a row.
    """
    speech_to_text = [
        _local_speech_option(LOCAL_STT_ID, LOCAL_STT_ENGINE, "Local faster-whisper")
    ]
    text_to_speech = [
        _local_speech_option(LOCAL_TTS_ID, LOCAL_TTS_ENGINE, "Local Supertone")
    ]
    for row in Provider.query.order_by(Provider.id).all():
        if has_capability(row.capabilities, "stt"):
            speech_to_text.append(_provider_speech_option(row))
        if has_capability(row.capabilities, "tts"):
            text_to_speech.append(_provider_speech_option(row))
    return {
        "speechToText": speech_to_text,
        "textToSpeech": text_to_speech,
        "liveVoiceNote": LIVE_VOICE_NOTE,
    }


def selectable_providers() -> List[Provider]:
    """Rows a personality may be pointed at. Image support is the filter."""
    rows = Provider.query.order_by(Provider.id).all()
    return [row for row in rows if has_images_capability(row.capabilities)]


def resolve_provider(provider_ref: str) -> Provider:
    """Turn a stored reference into the current row.

    'default' is looked up from is_default. Any other reference is that row's
    id. Changing the default does not rewrite personalities that store it.
    """
    from provider_registry import DEFAULT_PROVIDER_REF

    if provider_ref == DEFAULT_PROVIDER_REF:
        return get_default_provider()
    return Provider.query.filter_by(id=int(provider_ref)).one()
