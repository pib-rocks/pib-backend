from marshmallow import fields
from model.motor_model import Motor
from schema.controller_schema import controller_schema
from schema.sql_auto_with_camel_case_schema import SQLAutoWithCamelCaseSchema


class MotorSchemaSQLAutoWith(SQLAutoWithCamelCaseSchema):
    class Meta:
        model = Motor
        exclude = ("id", "controller_id")

    controller = fields.Nested(controller_schema)


motor_schema = MotorSchemaSQLAutoWith()
motors_schema = MotorSchemaSQLAutoWith(many=True)
motor_settings_schema = MotorSchemaSQLAutoWith(
    exclude=("controller", "controller_id", "channel")
)
motor_controller_schema = MotorSchemaSQLAutoWith(only=("name", "controller", "channel"))
