from model.assistant_model import AssistantModel
from app.app import ma


class AssistantSchema(ma.SQLAlchemyAutoSchema):
    class Meta:
        model = AssistantModel


assistant_schema = AssistantSchema()
assistants_schema = AssistantSchema(many=True)
