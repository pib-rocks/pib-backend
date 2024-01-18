from model.motor_model import Motor
from app.app import ma
from marshmallow import fields

from schema.bricklet_pin_schema import bricklet_pins_schema

class MotorSchema(ma.SQLAlchemyAutoSchema):
    class Meta:
        model = Motor
        exclude = ('id',)

    brickletPins = fields.Nested(bricklet_pins_schema)

motor_schema = MotorSchema()
motors_schema = MotorSchema(many=True)