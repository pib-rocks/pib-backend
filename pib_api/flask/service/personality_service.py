from typing import Any, List
from model.personality_model import Personality
from app.app import db


def get_all_personalities() -> List[Personality]:
    return Personality.query.all()


def get_personality(personality_id: str) -> Personality:
    return Personality.query.filter(Personality.personality_id == personality_id).one()


def create_personality(personality_dto: Any) -> List[Personality]:
    personality = Personality(
        name=personality_dto["name"],
        gender=personality_dto["gender"],
        pause_threshold=personality_dto["pause_threshold"],
        message_history=personality_dto["message_history"],
        assistant_model_id=personality_dto["assistant_model_id"],
    )
    db.session.add(personality)
    db.session.flush()
    return personality


def update_personality(personality_id: str, personality_dto: Any) -> Personality:
    personality = get_personality(personality_id)
    personality.name = personality_dto["name"]
    personality.gender = personality_dto["gender"].title()
    personality.pause_threshold = personality_dto["pause_threshold"]
    personality.message_history = (personality_dto["message_history"],)
    if "description" in personality_dto:
        personality.description = personality_dto["description"]
    personality.assistant_model_id = personality_dto["assistant_model_id"]
    db.session.flush()
    return personality


def delete_personality(personality_id: str) -> None:
    db.session.delete(get_personality(personality_id))
    db.session.flush()
