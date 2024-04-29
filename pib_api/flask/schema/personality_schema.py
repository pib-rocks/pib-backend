from model.personality_model import Personality
from pib_api.flask.schema.sql_auto_with_camel_case_schema import SQLAutoWithCamelCaseSchema


class PersonalitySchemaSQLAutoWith(SQLAutoWithCamelCaseSchema):
    class Meta:
        model = Personality
        include_fk = True


personality_schema = PersonalitySchemaSQLAutoWith(exclude=('id',))
upload_personality_schema = PersonalitySchemaSQLAutoWith(exclude=('id', 'personality_id'))
personalities_schema = PersonalitySchemaSQLAutoWith(exclude=('id',), many=True)
