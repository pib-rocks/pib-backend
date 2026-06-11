from marshmallow import fields

from model.button_program_model import ButtonProgram
from schema.sql_auto_with_camel_case_schema import SQLAutoWithCamelCaseSchema


class ButtonProgramLoadSchema(SQLAutoWithCamelCaseSchema):
    class Meta:
        model = ButtonProgram
        exclude = ("id", "bricklet_id", "program_id")

    brickletNumber = fields.Integer(required=True)
    programNumber = fields.String(allow_none=True)


button_programs_load_schema = ButtonProgramLoadSchema(many=True)


class ButtonProgramDumpSchema(SQLAutoWithCamelCaseSchema):
    class Meta:
        model = ButtonProgram
        exclude = ("id", "bricklet_id", "program_id")

    brickletNumber = fields.Integer(attribute="bricklet.bricklet_number")
    brickletUid = fields.String(attribute="bricklet.uid")
    programNumber = fields.Method("get_program_number", allow_none=True)

    def get_program_number(self, obj):
        return obj.program.program_number if obj.program else None


button_programs_schema = ButtonProgramDumpSchema(many=True)
