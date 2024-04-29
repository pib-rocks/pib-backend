from model.assistant_model import AssistantModel
from pib_api.flask.schema.sql_auto_with_camel_case_schema import SQLAutoWithCamelCaseSchema


class AssistantSchemaSQLAutoWith(SQLAutoWithCamelCaseSchema):
    class Meta:
        model = AssistantModel


assistant_model_schema = AssistantSchemaSQLAutoWith()
assistant_models_schema = AssistantSchemaSQLAutoWith(many=True)
