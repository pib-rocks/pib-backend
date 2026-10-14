import logging
import os
from typing import Any, List, Optional

from marshmallow import ValidationError

from model.personality_model import (
    DEFAULT_GENDER,
    DEFAULT_MESSAGE_HISTORY,
    DEFAULT_PAUSE_THRESHOLD,
    Personality,
)
from model.provider_model import RegistryModel
from app.app import db
from pib_hermes_config import (
    DEFAULT_PERSONALITY_REASONING_EFFORT,
    REASONING_EFFORT_ERROR,
    REASONING_EFFORTS,
    build_default_soul_text,
)
from pib_hermes_config.channel import (
    CHANNEL_DIRECT,
    CHANNEL_SMART,
    CHANNELS,
    smart_chats_enabled,
)
from pib_hermes_config.local_model import OFFLINE_CAPABILITY
from pib_hermes_config.live_interaction import personality_requests_actuation
from pib_hermes_config.live_session import (
    DEFAULT_LIVE_IDLE_TIMEOUT_SECONDS,
    VOICE_MODE_LIVE,
    VOICE_MODE_TURN_BASED,
    normalize_idle_timeout,
    voice_mode_for_model,
)
from pib_hermes_config.memory import write_memory
from pib_hermes_config.turn_taking import normalize_thinking_filler
from pib_hermes_config.voice_backends import (
    LOCAL_STT_ID,
    LOCAL_TTS_ID,
    normalize_stt_choice,
    normalize_tts_choice,
)
from provider_registry import (
    DEFAULT_PROVIDER_REF,
    MODEL_REF_CONFLICT_ERROR,
    MODEL_REF_FORM_ERROR,
    MODEL_REF_PROVIDER_GONE_ERROR,
    MODEL_REF_UNKNOWN_ERROR,
    has_capability,
    typed_model_id,
)
from service import provider_service, soul_service

#: Path of the daemon endpoint that owns the Hermes profile factory.
DAEMON_PROFILE_PATH = "/profile"
DEFAULT_DAEMON_URL = "http://ros-voice-assistant:8088"
# Sentinel: the caller did not name a reasoning level, so the payload omits it.
_REASONING_OMITTED = object()


def _daemon_profile_url() -> str:
    """URL of the daemon endpoint that provisions profiles.

    ``PIB_HERMES_DAEMON_URL`` is set by docker-compose for this service.
    """
    return os.environ.get("PIB_HERMES_DAEMON_URL") or DEFAULT_DAEMON_URL


def _provision_profile(
    personality_id: str,
    personality_name: Optional[str] = None,
    soul_text: Optional[str] = None,
    model: Optional[str] = None,
    provider: Optional[str] = None,
    endpoint_base: Optional[str] = None,
    reasoning_effort: Any = _REASONING_OMITTED,
    timeout: int = 60,
) -> dict:
    """Ask the Hermes daemon to create or repair a personality's Hermes profile.

    ``model``/``provider``/``endpoint_base`` are the personality's chosen model
    row (its api_name, the provider account's name and endpoint). The daemon
    writes them into the profile's config.yaml, so the smart chat runs the
    personality's model instead of a pinned default (PR-1930b).

    ``reasoning_effort`` is the personality column. A string is written to
    ``agent.reasoning_effort``. ``None`` means unmanaged and the profile's
    existing level is left as it is. Omitting the argument leaves the key
    out of the payload.

    Deliberately a plain HTTP call: the client package cannot be imported in this
    image (`public_api_client.__init__` requires the tryb configuration at import
    time) and only the daemon container can run the canonical Hermes profile
    factory. A failure is raised so the caller can surface it loudly.
    """
    import requests

    payload: dict = {"personality_id": personality_id}
    if personality_name is not None:
        payload["personality_name"] = personality_name
    if soul_text is not None:
        payload["soul_text"] = soul_text
    if model is not None:
        payload["model"] = model
    if provider is not None:
        payload["provider"] = provider
    if endpoint_base is not None:
        payload["endpoint_base"] = endpoint_base
    # Present even when null. Null tells the daemon the personality is
    # unmanaged and the profile's agent.reasoning_effort must stay as it is.
    if reasoning_effort is not _REASONING_OMITTED:
        payload["reasoning_effort"] = reasoning_effort

    url = _daemon_profile_url().rstrip("/") + DAEMON_PROFILE_PATH
    try:
        response = requests.post(url, json=payload, timeout=timeout)
    except requests.exceptions.RequestException as exc:
        raise RuntimeError(
            f"Hermes profile daemon is unreachable at {url}: {exc}"
        ) from exc

    try:
        result = response.json()
    except ValueError as exc:
        raise RuntimeError("Hermes profile daemon returned invalid JSON") from exc
    if (
        response.status_code >= 300
        or not isinstance(result, dict)
        or not result.get("ok")
    ):
        detail = result.get("error") if isinstance(result, dict) else None
        raise RuntimeError(
            f"Hermes profile provisioning failed ({response.status_code}): {detail}"
        )
    return result


def _ensure_description_from_soul(personality: Personality) -> bool:
    """Backfill empty description from SOUL.md. Returns True if updated."""
    if personality.description and personality.description.strip():
        return False
    soul = soul_service.read_soul(personality.personality_id)
    if not soul:
        return False
    personality.description = soul
    return True


def get_all_personalities() -> List[Personality]:
    personalities = Personality.query.all()
    updated = False
    for personality in personalities:
        if _ensure_description_from_soul(personality):
            updated = True
    if updated:
        db.session.flush()
    return personalities


def get_personality(personality_id: str) -> Personality:
    personality = Personality.query.filter(
        Personality.personality_id == personality_id
    ).one()
    if _ensure_description_from_soul(personality):
        db.session.flush()
    return personality


# 'default', or the decimal id of a model row. The catalogue api_name
# is not a reference: gemini-3.8-flash names a catalogue line, not a row.
# The provider follows from the model.
PROVIDER_REF_ERROR = "Provider reference must be 'default' or the id of a model row."


def _validated_model_row(model_id: int) -> RegistryModel:
    """The model row a typed reference names, or a 400 naming modelRef.

    Covers both that the row exists and that it belongs to a provider: a
    model whose provider account is gone is not a usable reference either.
    """
    row = RegistryModel.query.filter_by(id=model_id).first()
    if row is None:
        raise ValidationError({"modelRef": [MODEL_REF_UNKNOWN_ERROR]})
    if row.provider is None:
        raise ValidationError({"modelRef": [MODEL_REF_PROVIDER_GONE_ERROR]})
    return row


def _store_model_row(personality: Personality, model_id: int) -> None:
    """Store a concrete model row. The column keeps the bare row id."""
    _validated_model_row(model_id)
    personality.provider_ref = str(model_id)
    personality.assistant_model_id = model_id


def _store_policy(personality: Personality) -> None:
    """'default' follows the current default model; it is not an id."""
    personality.provider_ref = DEFAULT_PROVIDER_REF
    personality.assistant_model_id = None


def _store_provider_ref(personality: Personality, ref: str) -> None:
    """DEPRECATED path: persist a bare reference ('default' or a bare id).

    This is the old spelling of modelRef, accepted for the migration window.
    """
    if ref == DEFAULT_PROVIDER_REF:
        _store_policy(personality)
        return
    try:
        model_id = int(ref)
    except (TypeError, ValueError) as exc:
        raise ValidationError({"providerRef": [PROVIDER_REF_ERROR]}) from exc
    if model_id < 1 or RegistryModel.query.filter_by(id=model_id).first() is None:
        raise ValidationError({"providerRef": [PROVIDER_REF_ERROR]})
    personality.provider_ref = str(model_id)
    personality.assistant_model_id = model_id


def _typed_choice(personality_dto: Any) -> Optional[tuple]:
    """(kind, id) for a typed modelRef, or None when none was sent.

    'default' is the policy, everything else must be 'model:<id>'. A bare id
    or an unknown namespace raises: the kind must be stated, never guessed.
    """
    raw = personality_dto.get("model_ref")
    if raw is None or str(raw).strip() == "":
        return None
    text = str(raw).strip()
    if text == DEFAULT_PROVIDER_REF:
        return ("default", None)
    try:
        return ("model", typed_model_id(text))
    except ValueError as exc:
        raise ValidationError({"modelRef": [MODEL_REF_FORM_ERROR]}) from exc


def _legacy_alias_models(personality_dto: Any) -> set:
    """The models the deprecated aliases name, for the conflict check.

    ``None`` stands for the 'default' policy. A malformed alias alongside a
    typed reference is itself a conflict.
    """
    models: set = set()
    raw_ref = personality_dto.get("provider_ref")
    if raw_ref is not None and str(raw_ref).strip() != "":
        text = str(raw_ref).strip()
        if text == DEFAULT_PROVIDER_REF:
            models.add(None)
        elif text.isdigit() and int(text) >= 1:
            models.add(int(text))
        else:
            raise ValidationError({"modelRef": [MODEL_REF_CONFLICT_ERROR]})
    raw_id = personality_dto.get("assistant_model_id")
    if raw_id is not None:
        try:
            models.add(int(raw_id))
        except (TypeError, ValueError) as exc:
            raise ValidationError({"modelRef": [MODEL_REF_CONFLICT_ERROR]}) from exc
    return models


def _apply_provider_choice(
    personality: Personality, personality_dto: Any, *, creating: bool
) -> None:
    """Store the chosen model.

    The canonical field is ``modelRef``, a typed reference: ``"default"`` or
    ``"model:<id>"``. The prefix carries the kind, so a provider-account id
    can never be read as a model-row id. ``providerRef`` (bare) and
    ``assistantModelId`` are deprecated aliases that keep working for the
    migration window; a typed reference and an alias that disagree are
    rejected instead of silently resolved, and the deprecated spelling that
    is no longer needed disappears in its named removal release.
    """
    choice = _typed_choice(personality_dto)
    if choice is not None:
        alias_models = _legacy_alias_models(personality_dto)
        if alias_models and choice[1] not in alias_models:
            raise ValidationError({"modelRef": [MODEL_REF_CONFLICT_ERROR]})
        if choice[0] == "default":
            _store_policy(personality)
        else:
            _store_model_row(personality, choice[1])
        return

    # Deprecated, alias-only path. Precedence is the documented one the
    # interim fix (PR #412) set: an explicit assistantModelId wins.
    model_id = personality_dto.get("assistant_model_id")
    provider_ref = personality_dto.get("provider_ref")
    if model_id is not None:
        ref = str(model_id)
    elif provider_ref:
        ref = str(provider_ref)
    elif creating:
        ref = DEFAULT_PROVIDER_REF
    else:
        return
    _store_provider_ref(personality, ref)


def _apply_channel(
    personality: Personality, personality_dto: Any, *, creating: bool
) -> None:
    """Store the channel. Does not touch the identity text or MEMORY.md.

    With Hermes disabled, Smart cannot be stored. A create that omits the
    channel then stores Direct, which is the only path. When the chosen
    model carries the offline capability, a client that asks for Smart is
    refused, because that model does not provide the context window the
    agent requires.
    """
    channel_sent = "channel" in personality_dto and personality_dto["channel"]
    if channel_sent:
        requested = str(personality_dto["channel"])
    elif creating:
        requested = CHANNEL_DIRECT if not smart_chats_enabled() else CHANNEL_SMART
    else:
        return
    if requested not in CHANNELS:
        raise ValidationError({"channel": ["Channel must be smart or direct."]})
    if requested == CHANNEL_SMART and not smart_chats_enabled():
        raise ValidationError(
            {"channel": ["Smart chats are not available on this robot."]}
        )
    # An omitted channel on create still stores the default. Only a channel
    # the client sent is refused for a model with the offline capability.
    if (
        channel_sent
        and requested == CHANNEL_SMART
        and _model_has_offline_capability(personality)
    ):
        raise ValidationError(
            {
                "channel": [
                    "Smart chats are not available for the on-device model "
                    "because it does not provide the context window the agent "
                    "requires."
                ]
            }
        )
    personality.channel = requested


def _model_has_offline_capability(personality: Personality) -> bool:
    """True when the stored model row carries the offline capability.

    The row is the one ``_apply_provider_choice`` already stored. ``default``
    is a policy rather than a row, so this read does not resolve it and does
    not probe the device.
    """
    ref = getattr(personality, "provider_ref", None)
    if ref is None or str(ref) == DEFAULT_PROVIDER_REF:
        return False
    try:
        model_id = int(str(ref))
    except (TypeError, ValueError):
        return False
    row = RegistryModel.query.filter_by(id=model_id).first()
    if row is None:
        return False
    return has_capability(row.capabilities, OFFLINE_CAPABILITY)


def _provider_has(capability: str):
    def check(model_id: int) -> bool:
        row = RegistryModel.query.filter_by(id=model_id).first()
        if row is None:
            return False
        return has_capability(row.capabilities, capability)

    return check


def _apply_voice_backends(
    personality: Personality, personality_dto: Any, *, creating: bool
) -> None:
    """Local faster-whisper and Supertone, or a provider with that capability."""
    if creating or "stt_engine" in personality_dto:
        raw = (
            personality_dto.get("stt_engine")
            if "stt_engine" in personality_dto
            else LOCAL_STT_ID
        )
        try:
            personality.stt_engine = normalize_stt_choice(raw, _provider_has("stt"))
        except ValueError as exc:
            raise ValidationError({"sttEngine": [str(exc)]}) from exc
    if creating or "tts_engine" in personality_dto:
        raw = (
            personality_dto.get("tts_engine")
            if "tts_engine" in personality_dto
            else LOCAL_TTS_ID
        )
        try:
            personality.tts_engine = normalize_tts_choice(raw, _provider_has("tts"))
        except ValueError as exc:
            raise ValidationError({"ttsEngine": [str(exc)]}) from exc


def _store_derived_voice_mode(personality: Personality) -> None:
    """Store the mode of the chosen model. The client does not send one."""
    ref = getattr(personality, "provider_ref", None)
    model = provider_service.find_model(str(ref)) if ref else None
    if model is None:
        personality.voice_mode = VOICE_MODE_TURN_BASED
        return
    personality.voice_mode = voice_mode_for_model(
        has_capability(model.capabilities, "live"), model.api_name
    )


def _apply_live_chat_settings(
    personality: Personality, personality_dto: Any, *, creating: bool
) -> None:
    """The idle timeout that stops an unused live session.

    Voice mode is not taken from the client. It follows the chosen model.
    """
    if creating or "live_idle_timeout" in personality_dto:
        raw = (
            personality_dto.get("live_idle_timeout")
            if "live_idle_timeout" in personality_dto
            else DEFAULT_LIVE_IDLE_TIMEOUT_SECONDS
        )
        try:
            personality.live_idle_timeout = normalize_idle_timeout(raw)
        except ValueError as exc:
            raise ValidationError({"liveIdleTimeout": [str(exc)]}) from exc


def _reject_actuation_request(personality_dto: Any) -> None:
    """The actuation gate is PIB_MCP_ENABLE_ACTUATION, not a personality field."""
    if isinstance(personality_dto, dict) and personality_requests_actuation(
        personality_dto
    ):
        raise ValidationError(
            {"actuation": ["The actuation gate is an installation setting."]}
        )


def _apply_memory(personality: Personality, personality_dto: Any) -> None:
    """Store experience. Does not touch the character text or SOUL.md."""
    if "memory" not in personality_dto:
        return
    text = personality_dto.get("memory")
    if text is None:
        text = ""
    if not isinstance(text, str):
        raise ValidationError({"memory": ["Memory must be text."]})
    write_memory(personality.personality_id, text)


def _apply_reasoning_effort(
    personality: Personality, personality_dto: Any, *, creating: bool
) -> None:
    """Store the reasoning level. NULL leaves the Hermes profile alone.

    A create that omits the field stores "none". An unknown level is rejected.
    """
    if "reasoning_effort" not in personality_dto:
        if creating:
            personality.reasoning_effort = DEFAULT_PERSONALITY_REASONING_EFFORT
        return
    value = personality_dto.get("reasoning_effort")
    if value is None:
        personality.reasoning_effort = None
        return
    if not isinstance(value, str) or value not in REASONING_EFFORTS:
        raise ValidationError({"reasoningEffort": [REASONING_EFFORT_ERROR]})
    personality.reasoning_effort = value


def _apply_thinking_filler(personality: Personality, personality_dto: Any) -> None:
    if "thinking_filler" not in personality_dto:
        return
    personality.thinking_filler = normalize_thinking_filler(
        personality_dto.get("thinking_filler")
    )


def _tool_calling_value(personality_dto: Any, default: bool) -> bool:
    if "tool_calling" not in personality_dto:
        return default
    return bool(personality_dto["tool_calling"])


def _model_provisioning_fields(personality: Personality) -> dict:
    """Hermes profile fields for this personality's chosen model.

    ``{model, provider, endpoint_base}`` from the resolved row, so the daemon
    writes the model the personality is set to into its Hermes config.yaml.
    Empty when the row is gone or the personality follows a default with no
    row: the profile is then left exactly as it is.
    """
    model = provider_service.find_model(getattr(personality, "provider_ref", None))
    if model is None or not model.api_name:
        return {}
    fields: dict = {"model": model.api_name}
    provider = model.provider
    if provider is not None and provider.name:
        fields["provider"] = provider.name
        if provider.endpoint_base:
            fields["endpoint_base"] = provider.endpoint_base
    return fields


def create_personality(personality_dto: Any) -> Personality:
    _reject_actuation_request(personality_dto)
    # Only the name is required. The rest is defaulted here and edited later.
    personality = Personality(
        name=personality_dto["name"],
        gender=personality_dto.get("gender") or DEFAULT_GENDER,
        pause_threshold=personality_dto.get("pause_threshold", DEFAULT_PAUSE_THRESHOLD),
        message_history=personality_dto.get("message_history", DEFAULT_MESSAGE_HISTORY),
        stt_engine=LOCAL_STT_ID,
        tts_engine=LOCAL_TTS_ID,
        tool_calling=_tool_calling_value(personality_dto, True),
        voice_mode=VOICE_MODE_LIVE,
        live_idle_timeout=DEFAULT_LIVE_IDLE_TIMEOUT_SECONDS,
    )
    _apply_live_chat_settings(personality, personality_dto, creating=True)
    _apply_voice_backends(personality, personality_dto, creating=True)
    _apply_thinking_filler(personality, personality_dto)
    _apply_provider_choice(personality, personality_dto, creating=True)
    _store_derived_voice_mode(personality)
    _apply_channel(personality, personality_dto, creating=True)
    _apply_reasoning_effort(personality, personality_dto, creating=True)
    custom = ""
    if "description" in personality_dto and personality_dto["description"]:
        custom = str(personality_dto["description"]).strip()
    # Always seed the full SOUL.md (identity + optional custom + MCP docs) so
    # Cerebra's SOUL.md editor receives complete content, not an empty placeholder.
    soul_content = build_default_soul_text(personality.name, custom or None)
    personality.description = soul_content
    db.session.add(personality)
    db.session.flush()
    try:
        _provision_profile(
            personality.personality_id,
            personality_name=personality.name,
            soul_text=custom or None,
            reasoning_effort=personality.reasoning_effort,
            **_model_provisioning_fields(personality),
        )
        personality.profile_provisioned = True
    except Exception as exc:
        personality.profile_provisioned = False
        logging.error(
            "personality %s was created but its Hermes profile was not provisioned: %s",
            personality.personality_id,
            exc,
        )
    _apply_memory(personality, personality_dto)
    return personality


def update_personality(personality_id: str, personality_dto: Any) -> Personality:
    _reject_actuation_request(personality_dto)
    personality = get_personality(personality_id)
    name_changed = False
    if "name" in personality_dto:
        name_changed = personality.name != personality_dto["name"]
        personality.name = personality_dto["name"]
    if "gender" in personality_dto and personality_dto["gender"]:
        personality.gender = personality_dto["gender"].title()
    if "pause_threshold" in personality_dto:
        personality.pause_threshold = personality_dto["pause_threshold"]
    if "message_history" in personality_dto:
        personality.message_history = personality_dto["message_history"]
    if "description" in personality_dto:
        personality.description = personality_dto["description"]
        soul_service.write_soul(
            personality.personality_id,
            personality.description,
            personality_name=personality.name,
        )
    previous_model_ref = getattr(personality, "provider_ref", None)
    _apply_provider_choice(personality, personality_dto, creating=False)
    _store_derived_voice_mode(personality)
    model_changed = getattr(personality, "provider_ref", None) != previous_model_ref
    previous_effort = personality.reasoning_effort
    _apply_reasoning_effort(personality, personality_dto, creating=False)
    effort_changed = personality.reasoning_effort != previous_effort
    if name_changed or model_changed or effort_changed:
        try:
            _provision_profile(
                personality.personality_id,
                personality_name=personality.name,
                soul_text=personality.description,
                reasoning_effort=personality.reasoning_effort,
                **_model_provisioning_fields(personality),
            )
            personality.profile_provisioned = True
        except Exception as exc:
            personality.profile_provisioned = False
            logging.error(
                "personality %s updated but its Hermes profile was not provisioned: %s",
                personality.personality_id,
                exc,
            )
    _apply_channel(personality, personality_dto, creating=False)
    if "tool_calling" in personality_dto:
        personality.tool_calling = bool(personality_dto["tool_calling"])
    _apply_live_chat_settings(personality, personality_dto, creating=False)
    _apply_voice_backends(personality, personality_dto, creating=False)
    _apply_thinking_filler(personality, personality_dto)
    _apply_memory(personality, personality_dto)
    db.session.flush()
    return personality


def append_soul_lesson(personality_id: str, lesson: str) -> Personality:
    """Append one durable lesson without replacing any existing SOUL text."""
    personality = get_personality(personality_id)
    existing = personality.description or ""
    separator = "" if not existing or existing.endswith("\n") else "\n"
    personality.description = existing + separator + lesson
    soul_service.write_soul(personality.personality_id, personality.description)
    db.session.flush()
    return personality


def delete_personality(personality_id: str) -> None:
    db.session.delete(get_personality(personality_id))
    db.session.flush()
