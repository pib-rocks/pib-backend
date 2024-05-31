from model.bricklet_model import Bricklet
from schema.sql_auto_with_camel_case_schema import SQLAutoWithCamelCaseSchema


class BrickletSchemaSQLAutoWith(SQLAutoWithCamelCaseSchema):
    class Meta:
        model = Bricklet
        exclude = ("id",)


bricklet_schema = BrickletSchemaSQLAutoWith()
bricklets_schema = BrickletSchemaSQLAutoWith(many=True)
bricklet_uid_only_schema = BrickletSchemaSQLAutoWith(only=("uid",))
