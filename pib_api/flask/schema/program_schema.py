from model.program_model import Program
from pib_api.flask.schema.sql_auto_with_camel_case_schema import SQLAutoWithCamelCaseSchema


class ProgramSchemaSQLAutoWith(SQLAutoWithCamelCaseSchema):
    class Meta:
        model = Program


program_schema_name_only = ProgramSchemaSQLAutoWith(only=('name',))
program_schema_without_program = ProgramSchemaSQLAutoWith(only=('name', 'program_number'))
programs_schema_without_program = ProgramSchemaSQLAutoWith(only=('name', 'program_number'), many=True)
