from marshmallow import fields, validate
from model.personality_model import Personality
from pib_hermes_config.channel import (
    CHANNEL_DIRECT,
    CHANNEL_SMART,
    effective_channel,
    smart_chats_enabled,
)
from schema.sql_auto_with_camel_case_schema import SQLAutoWithCamelCaseSchema
from service import soul_service


class PersonalitySchemaSQLAutoWith(SQLAutoWithCamelCaseSchema):
    class Meta:
        model = Personality
        include_fk = True

    stt_engine = fields.String(
        validate=validate.OneOf(["local_whisper", "tryb_api"]),
        dump_default="local_whisper",
        load_default="local_whisper",
    )
    assistant_model_id = fields.Integer(required=False, allow_none=True)
    provider_ref = fields.String(required=False, allow_none=True)
    channel = fields.String(
        required=False,
        validate=validate.OneOf([CHANNEL_SMART, CHANNEL_DIRECT]),
    )
    effective_channel = fields.Method("get_effective_channel", dump_only=True)
    smart_chats_enabled = fields.Method("get_smart_chats_enabled", dump_only=True)
    soul_path = fields.Method("get_soul_path", dump_only=True)
    profile_provisioned = fields.Boolean(dump_only=True)

    def get_soul_path(self, obj: Personality) -> str:
        return soul_service.soul_path_for(obj.personality_id)

    def get_effective_channel(self, obj: Personality) -> str:
        return effective_channel(obj.channel)

    def get_smart_chats_enabled(self, _obj: Personality) -> bool:
        return smart_chats_enabled()


personality_schema = PersonalitySchemaSQLAutoWith(exclude=("id",))
upload_personality_schema = PersonalitySchemaSQLAutoWith(
    exclude=("id", "personality_id")
)
update_personality_schema = PersonalitySchemaSQLAutoWith(
    exclude=("id", "personality_id"), partial=True
)
personalities_schema = PersonalitySchemaSQLAutoWith(exclude=("id",), many=True)
