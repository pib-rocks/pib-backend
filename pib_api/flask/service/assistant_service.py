from model.assistant_model import AssistantModel
from app.app import db


def get_all_assistants() -> list[AssistantModel]:
    return AssistantModel.query.all()


def get_assistant_by_id(assistant_id: int) -> AssistantModel:
    return AssistantModel.query.filter_by(id=assistant_id).first()
