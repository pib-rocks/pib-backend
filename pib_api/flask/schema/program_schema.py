from model.program_model import Program
from schema.SQLAutoWithCamelCaseSchema import SQLAutoWithCamelCaseSchema


class ProgramSchemaSQLAutoWith(SQLAutoWithCamelCaseSchema):
    class Meta:
        model = Program


program_schema_name_only = ProgramSchemaSQLAutoWith(only=('name',))
program_schema_without_program = ProgramSchemaSQLAutoWith(only=('name', 'program_number'))
programs_schema_without_program = ProgramSchemaSQLAutoWith(only=('name', 'program_number'), many=True)
