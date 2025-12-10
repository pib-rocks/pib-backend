from typing import List
from model.assistant_model import AssistantModel


def get_all_assistant_models() -> List[AssistantModel]:
    return AssistantModel.query.filter(
        AssistantModel.api_name != "gemini-2.5-flash"
    ).all()


def get_assistant_model_by_id(assistant_model_id: int) -> AssistantModel:
    return AssistantModel.query.filter_by(id=assistant_model_id).first()
