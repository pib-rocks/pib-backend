from marshmallow import fields
from model.motor_model import Motor
from schema.bricklet_pin_schema import bricklet_pins_schema
from schema.sql_auto_with_camel_case_schema import SQLAutoWithCamelCaseSchema


class MotorSchemaSQLAutoWith(SQLAutoWithCamelCaseSchema):
    class Meta:
        model = Motor
        exclude = ("id",)

    bricklet_pins = fields.Nested(bricklet_pins_schema)


motor_schema = MotorSchemaSQLAutoWith()
motors_schema = MotorSchemaSQLAutoWith(many=True)
motor_settings_schema = MotorSchemaSQLAutoWith(exclude=("bricklet_pins",))
motor_bricklet_pins_schema = MotorSchemaSQLAutoWith(only=("name", "bricklet_pins"))
