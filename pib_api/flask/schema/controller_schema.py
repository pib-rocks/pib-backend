from model.controller_model import Controller
from schema.sql_auto_with_camel_case_schema import SQLAutoWithCamelCaseSchema


class ControllerSchema(SQLAutoWithCamelCaseSchema):
    class Meta:
        model = Controller
        exclude = ("id", "created_at", "updated_at", "motors")


controller_schema = ControllerSchema()
controllers_schema = ControllerSchema(many=True)
