from model.personality_model import Personality
from schema.SQLAutoWithCamelCaseSchema import SQLAutoWithCamelCaseSchema


class PersonalitySchemaSQLAutoWith(SQLAutoWithCamelCaseSchema):
    class Meta:
        model = Personality
        include_fk = True


personality_schema = PersonalitySchemaSQLAutoWith(exclude=('id',))
upload_personality_schema = PersonalitySchemaSQLAutoWith(exclude=('id', 'personality_id'))
personalities_schema = PersonalitySchemaSQLAutoWith(exclude=('id',), many=True)
