from marshmallow import fields, validate
from model.personality_model import Personality
from schema.sql_auto_with_camel_case_schema import SQLAutoWithCamelCaseSchema


class PersonalitySchemaSQLAutoWith(SQLAutoWithCamelCaseSchema):
    class Meta:
        model = Personality
        include_fk = True

    stt_engine = fields.String(
        validate=validate.OneOf(["local_whisper", "tryb_api"]),
        dump_default="local_whisper",
        load_default="local_whisper",
    )


personality_schema = PersonalitySchemaSQLAutoWith(exclude=("id",))
upload_personality_schema = PersonalitySchemaSQLAutoWith(
    exclude=("id", "personality_id")
)
update_personality_schema = PersonalitySchemaSQLAutoWith(
    exclude=("id", "personality_id"), partial=True
)
personalities_schema = PersonalitySchemaSQLAutoWith(exclude=("id",), many=True)
