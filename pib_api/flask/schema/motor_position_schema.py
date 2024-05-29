from model.motor_position_model import MotorPosition
from schema.sql_auto_with_camel_case_schema import SQLAutoWithCamelCaseSchema


class MotorPositionSchema(SQLAutoWithCamelCaseSchema):
    class Meta:
        model = MotorPosition
        exclude = ('id',)

motor_positions_schema = MotorPositionSchema(only=('position', 'motorname'), many=True)