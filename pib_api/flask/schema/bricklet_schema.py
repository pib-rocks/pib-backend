from model.bricklet_model import Bricklet
from app.app import ma

class BrickletSchema(ma.SQLAlchemyAutoSchema):
    class Meta:
        model = Bricklet
        exclude = ('id',)

bricklet_schema = BrickletSchema()
bricklets_schema = BrickletSchema(many=True)
bricklet_uid_only_schema = BrickletSchema(only=('uid',))