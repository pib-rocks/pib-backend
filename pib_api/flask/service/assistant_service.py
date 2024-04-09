from model.assistant_model import AssistantModel
from app.app import db


def get_all_assistants() -> list[AssistantModel]:
    return AssistantModel.query.all()
