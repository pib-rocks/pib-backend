from marshmallow import fields, Schema

class ProgramCodeSchema(Schema):
    visual = fields.String(required=True)
    python = fields.String(required=True)

program_code_schema = ProgramCodeSchema()
program_code_visual_only_schema = ProgramCodeSchema(only=('visual',))