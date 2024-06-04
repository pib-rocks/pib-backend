from model.assistant_model import AssistantModel
from schema.sql_auto_with_camel_case_schema import SQLAutoWithCamelCaseSchema


class AssistantModelSchemaSQLAutoWith(SQLAutoWithCamelCaseSchema):
    class Meta:
        model = AssistantModel


assistant_model_schema = AssistantModelSchemaSQLAutoWith()
assistant_models_schema = AssistantModelSchemaSQLAutoWith(many=True)
