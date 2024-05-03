from marshmallow import fields, Schema

class ProgramCodeSchema(Schema):
    visual = fields.String(required=True)

program_code_schema = ProgramCodeSchema()