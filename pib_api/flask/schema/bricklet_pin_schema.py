from marshmallow import fields

from model.bricklet_pin_model import BrickletPin
from pib_api.flask.schema.sql_auto_with_camel_case_schema import SQLAutoWithCamelCaseSchema
from schema.bricklet_schema import bricklet_uid_only_schema


class BrickletPinSchemaSQLAutoWith(SQLAutoWithCamelCaseSchema):
    class Meta:
        model = BrickletPin
        exclude = ('id',)

    bricklet = fields.Pluck(bricklet_uid_only_schema, 'uid')


bricklet_pins_schema = BrickletPinSchemaSQLAutoWith(many=True)
