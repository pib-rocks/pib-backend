from model.program_model import Program
from schema.sql_auto_with_camel_case_schema import SQLAutoWithCamelCaseSchema


class ProgramSchemaSQLAutoWith(SQLAutoWithCamelCaseSchema):
    class Meta:
        model = Program


program_schema_name_only = ProgramSchema(only=("name",))
program_schema_without_code = ProgramSchema(only=("name", "program_number"))
programs_schema_without_code = ProgramSchema(only=("name", "program_number"), many=True)
program_schema_code_visual_only = ProgramSchema(only=("code_visual",))
