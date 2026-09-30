import logging
import os
from typing import Any, List, Optional

from marshmallow import ValidationError

from model.personality_model import Personality
from model.provider_model import Provider
from app.app import db
from pib_hermes_config import build_default_soul_text
from pib_hermes_config.channel import (
    CHANNEL_DIRECT,
    CHANNEL_SMART,
    CHANNELS,
    smart_chats_enabled,
)
from pib_hermes_config.voice_backends import (
    LOCAL_STT_ID,
    LOCAL_TTS_ID,
    normalize_stt_choice,
    normalize_tts_choice,
)
from provider_registry import DEFAULT_PROVIDER_REF, has_capability
from service import soul_service

#: Path of the daemon endpoint that owns the Hermes profile factory.
DAEMON_PROFILE_PATH = "/profile"
DEFAULT_DAEMON_URL = "http://ros-voice-assistant:8088"


def _daemon_profile_url() -> str:
    """URL of the daemon endpoint that provisions profiles.

    ``PIB_HERMES_DAEMON_URL`` is set by docker-compose for this service.
    """
    return os.environ.get("PIB_HERMES_DAEMON_URL") or DEFAULT_DAEMON_URL


def _provision_profile(
    personality_id: str,
    personality_name: Optional[str] = None,
    soul_text: Optional[str] = None,
    timeout: int = 60,
) -> dict:
    """Ask the Hermes daemon to create or repair a personality's Hermes profile.

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


def _store_provider_ref(personality: Personality, ref: str) -> None:
    """Persist a provider pointer. 'default' is not resolved into an id."""
    if ref == DEFAULT_PROVIDER_REF:
        personality.provider_ref = DEFAULT_PROVIDER_REF
        personality.assistant_model_id = None
        return
    try:
        provider_id = int(ref)
    except (TypeError, ValueError) as exc:
        raise ValidationError({"providerRef": ["Unknown provider reference."]}) from exc
    if provider_id < 1 or Provider.query.filter_by(id=provider_id).first() is None:
        raise ValidationError({"providerRef": ["Unknown provider reference."]})
    personality.provider_ref = str(provider_id)
    personality.assistant_model_id = provider_id


def _apply_channel(
    personality: Personality, personality_dto: Any, *, creating: bool
) -> None:
    """Store the channel. Does not touch the identity text or MEMORY.md.

    With Hermes disabled, Smart cannot be stored. A create that omits the
    channel then stores Direct, which is the only path.
    """
    if "channel" in personality_dto and personality_dto["channel"]:
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
    personality.channel = requested


def _apply_provider_choice(
    personality: Personality, personality_dto: Any, *, creating: bool
) -> None:
    provider_ref = personality_dto.get("provider_ref")
    model_id = personality_dto.get("assistant_model_id")
    if provider_ref:
        ref = str(provider_ref)
    elif model_id is not None:
        ref = str(model_id)
    elif creating:
        ref = DEFAULT_PROVIDER_REF
    else:
        return
    _store_provider_ref(personality, ref)


def _provider_has(capability: str):
    def check(provider_id: int) -> bool:
        row = Provider.query.filter_by(id=provider_id).first()
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


def _tool_calling_value(personality_dto: Any, default: bool) -> bool:
    if "tool_calling" not in personality_dto:
        return default
    return bool(personality_dto["tool_calling"])


def create_personality(personality_dto: Any) -> Personality:
    personality = Personality(
        name=personality_dto["name"],
        gender=personality_dto["gender"],
        pause_threshold=personality_dto["pause_threshold"],
        message_history=personality_dto["message_history"],
        stt_engine=LOCAL_STT_ID,
        tts_engine=LOCAL_TTS_ID,
        tool_calling=_tool_calling_value(personality_dto, True),
    )
    _apply_voice_backends(personality, personality_dto, creating=True)
    _apply_provider_choice(personality, personality_dto, creating=True)
    _apply_channel(personality, personality_dto, creating=True)
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
        )
        personality.profile_provisioned = True
    except Exception as exc:
        personality.profile_provisioned = False
        logging.error(
            "personality %s was created but its Hermes profile was not provisioned: %s",
            personality.personality_id,
            exc,
        )
    return personality


def update_personality(personality_id: str, personality_dto: Any) -> Personality:
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
    if name_changed:
        try:
            _provision_profile(
                personality.personality_id,
                personality_name=personality.name,
                soul_text=personality.description,
            )
            personality.profile_provisioned = True
        except Exception as exc:
            personality.profile_provisioned = False
            logging.error(
                "personality %s updated but its Hermes profile was not provisioned: %s",
                personality.personality_id,
                exc,
            )
    _apply_provider_choice(personality, personality_dto, creating=False)
    _apply_channel(personality, personality_dto, creating=False)
    if "tool_calling" in personality_dto:
        personality.tool_calling = bool(personality_dto["tool_calling"])
    _apply_voice_backends(personality, personality_dto, creating=False)
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
