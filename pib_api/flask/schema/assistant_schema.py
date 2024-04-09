from model.assistant_model import AssistantModel
from app.app import ma

class AssistantSchema(ma.SQLAlchemyAutoSchema):
    class Meta:
        model = AssistantModel
        exclude = ('id',)

assistant_schema = AssistantSchema()