from model.bricklet_model import Bricklet
from schema.SQLAutoWithCamelCaseSchema import SQLAutoWithCamelCaseSchema


class BrickletSchemaSQLAutoWith(SQLAutoWithCamelCaseSchema):
    class Meta:
        model = Bricklet
        exclude = ('id',)


bricklet_schema = BrickletSchemaSQLAutoWith()
bricklets_schema = BrickletSchemaSQLAutoWith(many=True)
bricklet_uid_only_schema = BrickletSchemaSQLAutoWith(only=('uid',))
