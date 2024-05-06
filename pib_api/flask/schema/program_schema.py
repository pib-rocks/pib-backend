from model.program_model import Program
from app.app import ma

class ProgramSchema(ma.SQLAlchemyAutoSchema):
    class Meta:
        model = Program

program_schema_name_only = ProgramSchema(only=('name',))
program_schema_without_code = ProgramSchema(only=('name', 'programNumber'))
programs_schema_without_code = ProgramSchema(only=('name', 'programNumber'), many=True)
program_schema_code_visual_only = ProgramSchema(only=('codeVisual',))