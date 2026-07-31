from typing import Any, List
from model.personality_model import Personality
from app.app import db
from service import soul_service


def get_all_personalities() -> List[Personality]:
    return Personality.query.all()


def get_personality(personality_id: str) -> Personality:
    return Personality.query.filter(Personality.personality_id == personality_id).one()


def create_personality(personality_dto: Any) -> Personality:
    personality = Personality(
        name=personality_dto["name"],
        gender=personality_dto["gender"],
        pause_threshold=personality_dto["pause_threshold"],
        message_history=personality_dto["message_history"],
        assistant_model_id=personality_dto["assistant_model_id"],
        stt_engine=personality_dto.get("stt_engine", "local_whisper"),
    )
    if "description" in personality_dto:
        personality.description = personality_dto["description"]
    db.session.add(personality)
    db.session.flush()
    soul_service.write_soul(personality.personality_id, personality.description)
    return personality


def update_personality(personality_id: str, personality_dto: Any) -> Personality:
    personality = get_personality(personality_id)
    if "name" in personality_dto:
        personality.name = personality_dto["name"]
    if "gender" in personality_dto and personality_dto["gender"]:
        personality.gender = personality_dto["gender"].title()
    if "pause_threshold" in personality_dto:
        personality.pause_threshold = personality_dto["pause_threshold"]
    if "message_history" in personality_dto:
        personality.message_history = personality_dto["message_history"]
    if "description" in personality_dto:
        personality.description = personality_dto["description"]
        soul_service.write_soul(personality.personality_id, personality.description)
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
