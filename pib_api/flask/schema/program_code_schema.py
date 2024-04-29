from marshmallow import fields

from schema.sql_auto_with_camel_case_schema import SQLAutoWithCamelCaseSchema


class ProgramCodeSchemaSQLAutoWith(SQLAutoWithCamelCaseSchema):
    visual = fields.String(required=True)
    python = fields.String(required=True)


program_code_schema = ProgramCodeSchemaSQLAutoWith()
program_code_visual_only_schema = ProgramCodeSchemaSQLAutoWith(only=('visual',))
