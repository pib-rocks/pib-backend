from model.bricklet_pin_model import BrickletPin
from schema.bricklet_schema import bricklet_uid_only_schema
from app.app import ma
from marshmallow import fields


class BrickletPinSchema(ma.SQLAlchemyAutoSchema):
    class Meta:
        model = BrickletPin
        exclude = ("id",)

    bricklet = fields.Pluck(bricklet_uid_only_schema, "uid")


bricklet_pins_schema = BrickletPinSchema(many=True)
