from model.assistant_model import AssistantModel
from schema.SQLAutoWithCamelCaseSchema import SQLAutoWithCamelCaseSchema


class AssistantSchemaSQLAutoWith(SQLAutoWithCamelCaseSchema):
    class Meta:
        model = AssistantModel


assistant_schema = AssistantSchemaSQLAutoWith()
assistants_schema = AssistantSchemaSQLAutoWith(many=True)
