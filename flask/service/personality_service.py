from typing import Any
from model.personality_model import Personality
from app.app import db


def get_personality(personality_id: str) -> Personality:
    return Personality.query.filter(Personality.personalityId == personality_id).one()