from typing import Any
from model.personality_model import Personality
from app.app import db
import uuid


def get_all_personalities() -> list[Personality]:
    return Personality.query.all()


def get_personality(personality_id: str) -> Personality:
    return Personality.query.filter(Personality.personality_id == personality_id).one()


def create_personality(personality_dto: dict[str, Any]) -> list[Personality]:
    personality = Personality(
        name=personality_dto['name'],
        gender=personality_dto['gender'],
        pause_threshold=personality_dto['pause_threshold'],

        assistant_id=personality_dto['assistant_id'])
    db.session.add(personality)
    db.session.flush()
    return personality


def update_personality(personality_id: str, personality_dto: dict[str, Any]) -> Personality:
    personality = get_personality(personality_id)
    personality.name = personality_dto['name']
    personality.gender = personality_dto['gender']
    personality.pause_threshold = personality_dto['pause_threshold']
    if 'description' in personality_dto:
        personality.description = personality_dto['description']
    personality.assistant_id = personality_dto['assistant_id']
    db.session.flush()
    return personality


def delete_personality(personality_id: str) -> None:
    db.session.delete(get_personality(personality_id))
    db.session.flush()
