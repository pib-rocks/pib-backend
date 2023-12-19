from model.motor_model import Motor
from app.app import ma

class MotorSchema(ma.SQLAlchemyAutoSchema):
    class Meta:
        model = Motor
        exclude = ('id', 'effort')

motor_schema = MotorSchema()
motors_schema = MotorSchema(many=True)