from marshmallow import fields

from model.pose_model import Pose
from schema.sql_auto_with_camel_case_schema import SQLAutoWithCamelCaseSchema
from schema.motor_position_schema import motor_positions_schema


class PoseSchema(SQLAutoWithCamelCaseSchema):
    class Meta:
        model = Pose
        exclude = ('id',)

    motor_positions = fields.Nested(motor_positions_schema)

poses_schema = PoseSchema(many=True, only=('pose_id', 'name'))
pose_schema = PoseSchema(only=('pose_id', 'name', 'motor_positions'))
create_pose_schema = PoseSchema(only=('name', 'motor_positions'))
pose_schema_motor_positions_only = PoseSchema(only=('motor_positions',))
pose_schema_name_only = PoseSchema(only=('name',))
pose_schema_without_motor_positions = PoseSchema(only=('pose_id', 'name'))