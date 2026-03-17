from dataclasses import fields

from model.button_program_model import ButtonProgram
from schema.sql_auto_with_camel_case_schema import SQLAutoWithCamelCaseSchema


class ButtonProgramSchemaSQLAutoWith(SQLAutoWithCamelCaseSchema):
    class Meta:
        model = ButtonProgram
        exclude = ("id", "bricklet_id", "program_id")

    brickletNumber = fields.Integer(attribute="bricklet.bricklet_number")
    programNumber = fields.Integer(attribute="program.program_number", allow_none=True)


button_program_schema = ButtonProgramSchemaSQLAutoWith()
button_programs_schema = ButtonProgramSchemaSQLAutoWith(many=True)
