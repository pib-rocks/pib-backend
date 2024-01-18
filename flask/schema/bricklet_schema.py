from model.bricklet_model import Bricklet
from app.app import ma
from marshmallow import fields

class BrickletSchema(ma.SQLAlchemyAutoSchema):
    class Meta:
        model = Bricklet
        exclude = ('id',)

bricklet_uid_only_schema = BrickletSchema(only=('uid',))