import logging
import os
from typing import Any, List, Optional
from model.personality_model import Personality
from app.app import db
from pib_hermes_config import build_default_soul_text
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


def create_personality(personality_dto: Any) -> Personality:
    personality = Personality(
        name=personality_dto["name"],
        gender=personality_dto["gender"],
        pause_threshold=personality_dto["pause_threshold"],
        message_history=personality_dto["message_history"],
        assistant_model_id=personality_dto["assistant_model_id"],
        stt_engine=personality_dto.get("stt_engine", "local_whisper"),
    )
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
    if "assistant_model_id" in personality_dto:
        personality.assistant_model_id = personality_dto["assistant_model_id"]
    if "stt_engine" in personality_dto:
        personality.stt_engine = personality_dto["stt_engine"]
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
