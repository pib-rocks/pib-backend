from typing import List, Optional

from model.assistant_model import AssistantModel
from model.provider_model import Provider, RegistryModel
from pib_hermes_config.voice_backends import (
    LIVE_VOICE_NOTE,
    LOCAL_STT_ENGINE,
    LOCAL_STT_ID,
    LOCAL_TTS_ENGINE,
    LOCAL_TTS_ID,
)
from provider_registry import (
    DEFAULT_PROVIDER_REF,
    capabilities_for,
    capabilities_held_by_all,
    has_capability,
    is_listed_model,
    is_registry_default,
    provider_name_for,
)


def sync_shared_capabilities(provider: Provider) -> None:
    """Store the flags that are true on every model of this provider."""
    rows = RegistryModel.query.filter_by(provider_id=provider.id).all()
    provider.capabilities = capabilities_held_by_all(row.capabilities for row in rows)


def _provider_for(api_name: str, visual_name: str) -> Provider:
    name = provider_name_for(api_name, visual_name)
    provider = Provider.query.filter_by(name=name).one_or_none()
    if provider is not None:
        return provider
    provider = Provider(
        name=name,
        endpoint_base=None,
        credential_ref=None,
        capabilities=capabilities_held_by_all(()),
    )
    from app.app import db

    db.session.add(provider)
    db.session.flush()
    return provider


def build_registry_model(model: AssistantModel) -> RegistryModel:
    """One model row for an assistant model, using that model's id.

    The provider is the catalogue account. A live model is its own row.
    Nothing stores a second id beside the one the catalogue names.
    """
    flags = capabilities_for(model.api_name, bool(model.has_image_support))
    provider = _provider_for(model.api_name, model.visual_name)
    return RegistryModel(
        id=model.id,
        provider_id=provider.id,
        api_name=model.api_name,
        visual_name=model.visual_name,
        has_image_support=bool(model.has_image_support),
        capabilities=flags,
        is_default=is_registry_default(model.api_name),
        live_model=None,
        live_model_checked_on=None,
    )


def attach_registry_models(models: List[AssistantModel]) -> List[RegistryModel]:
    """Add model rows and refresh each provider's shared flags."""
    from app.app import db

    rows = [build_registry_model(model) for model in models]
    db.session.add_all(rows)
    db.session.flush()
    seen: set[int] = set()
    for row in rows:
        if row.provider_id in seen:
            continue
        seen.add(row.provider_id)
        sync_shared_capabilities(row.provider)
    return rows


def get_provider_by_id(provider_id: int) -> Optional[Provider]:
    return Provider.query.filter_by(id=provider_id).first()


def get_model_by_id(model_id: int) -> Optional[RegistryModel]:
    return RegistryModel.query.filter_by(id=model_id).first()


def get_default_model() -> RegistryModel:
    return RegistryModel.query.filter_by(is_default=True).one()


def get_default_provider() -> Provider:
    """The provider of the current default model."""
    return get_default_model().provider


def _local_speech_option(option_id: str, engine: str, label: str) -> dict:
    return {
        "id": option_id,
        "kind": "local",
        "engine": engine,
        "label": label,
    }


def _model_speech_option(row: RegistryModel) -> dict:
    return {
        "id": str(row.id),
        "kind": "provider",
        "engine": row.api_name,
        "label": row.visual_name,
    }


def speech_backends() -> dict:
    """Local engines plus model rows that carry stt or tts.

    The image filter does not apply here. A model is offered for speech only
    when its own capability flag is set. ElevenLabs is not a row.
    """
    speech_to_text = [
        _local_speech_option(LOCAL_STT_ID, LOCAL_STT_ENGINE, "Local faster-whisper")
    ]
    text_to_speech = [
        _local_speech_option(LOCAL_TTS_ID, LOCAL_TTS_ENGINE, "Local Supertone")
    ]
    for row in RegistryModel.query.order_by(RegistryModel.id).all():
        if has_capability(row.capabilities, "stt"):
            speech_to_text.append(_model_speech_option(row))
        if has_capability(row.capabilities, "tts"):
            text_to_speech.append(_model_speech_option(row))
    return {
        "speechToText": speech_to_text,
        "textToSpeech": text_to_speech,
        "liveVoiceNote": LIVE_VOICE_NOTE,
    }


def selectable_models() -> List[RegistryModel]:
    """Models a personality may choose. An image model or a named live model."""
    rows = RegistryModel.query.order_by(RegistryModel.id).all()
    return [row for row in rows if is_listed_model(row)]


def providers_with_models() -> List[tuple]:
    """Each provider with the models a personality may choose.

    A chat model without images is left out. A live model stays, because
    choosing that model is how live speech is selected.
    """
    groups = []
    for provider in Provider.query.order_by(Provider.id).all():
        models = [
            row
            for row in sorted(provider.models, key=lambda row: row.id)
            if is_listed_model(row)
        ]
        if models:
            groups.append((provider, models))
    return groups


def resolve_model(provider_ref: str) -> RegistryModel:
    """Turn a stored reference into the current model.

    'default' is looked up from is_default. Any other reference is that
    model's id. Changing the default does not rewrite personalities that
    store it. The provider follows from the model.
    """
    if provider_ref == DEFAULT_PROVIDER_REF:
        return get_default_model()
    return RegistryModel.query.filter_by(id=int(provider_ref)).one()


def find_model(provider_ref: object) -> Optional[RegistryModel]:
    """The model a stored reference points at, or None when that row is gone.

    A removed model has no row and no status to read. The personality's
    reference is simply dangling, and that is what is detected here. It also
    covers a row that was deleted by hand.
    """
    if provider_ref == DEFAULT_PROVIDER_REF:
        return RegistryModel.query.filter_by(is_default=True).first()
    try:
        model_id = int(str(provider_ref))
    except (TypeError, ValueError):
        return None
    return RegistryModel.query.filter_by(id=model_id).first()


def resolve_provider(provider_ref: str) -> Provider:
    """The provider of the model a stored reference points at."""
    return resolve_model(provider_ref).provider


def find_provider(provider_ref: object) -> Optional[Provider]:
    """The provider of the referenced model, or None when that model is gone."""
    model = find_model(provider_ref)
    if model is None:
        return None
    return model.provider
