from __future__ import annotations

from marshmallow import fields, validate
from model.personality_model import Personality
from pib_hermes_config.channel import (
    CHANNEL_DIRECT,
    CHANNEL_SMART,
    effective_channel,
    smart_chats_enabled,
)
from pib_hermes_config.live_session import (
    VOICE_MODE_LIVE,
    VOICE_MODE_TURN_BASED,
    voice_start_mode,
)
from pib_hermes_config.voice_backends import live_voice_note, local_voice_applies
from provider_registry import has_capability
from schema.sql_auto_with_camel_case_schema import SQLAutoWithCamelCaseSchema
from service import provider_service, soul_service


class PersonalitySchemaSQLAutoWith(SQLAutoWithCamelCaseSchema):
    class Meta:
        model = Personality
        include_fk = True

    stt_engine = fields.String(
        required=False,
        dump_default="local_whisper",
        load_default="local_whisper",
    )
    tts_engine = fields.String(
        required=False,
        dump_default="supertone",
        load_default="supertone",
    )
    local_voice_applies = fields.Method("get_local_voice_applies", dump_only=True)
    live_voice_note = fields.Method("get_live_voice_note", dump_only=True)
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
    voice_mode = fields.String(
        required=False,
        validate=validate.OneOf([VOICE_MODE_LIVE, VOICE_MODE_TURN_BASED]),
    )
    live_idle_timeout = fields.Integer(required=False, validate=validate.Range(min=1))
    thinking_filler = fields.String(
        required=False,
        allow_none=True,
        validate=validate.Length(max=255),
    )
    live_model = fields.Method("get_live_model", dump_only=True)
    voice_start_mode = fields.Method("get_voice_start_mode", dump_only=True)

    def get_soul_path(self, obj: Personality) -> str:
        return soul_service.soul_path_for(obj.personality_id)

    def get_effective_channel(self, obj: Personality) -> str:
        return effective_channel(obj.channel)

    def get_smart_chats_enabled(self, _obj: Personality) -> bool:
        return smart_chats_enabled()

    def _resolved_provider(self, obj: Personality):
        ref = getattr(obj, "provider_ref", None)
        if not ref:
            return None
        try:
            return provider_service.resolve_provider(str(ref))
        except Exception:
            return None

    def get_live_model(self, obj: Personality) -> str | None:
        provider = self._resolved_provider(obj)
        if provider is None:
            return None
        return provider.live_model

    def get_voice_start_mode(self, obj: Personality) -> str:
        """What the one voice button will start: live or turn-based."""
        provider = self._resolved_provider(obj)
        capable = bool(provider) and has_capability(provider.capabilities, "live")
        model = provider.live_model if provider is not None else None
        mode = getattr(obj, "voice_mode", None) or VOICE_MODE_LIVE
        return voice_start_mode(mode, capable, model)

    def _provider_is_live(self, obj: Personality) -> bool:
        return self.get_voice_start_mode(obj) == VOICE_MODE_LIVE

    def get_local_voice_applies(self, obj: Personality) -> bool:
        return local_voice_applies(self._provider_is_live(obj))

    def get_live_voice_note(self, obj: Personality) -> str | None:
        return live_voice_note(self._provider_is_live(obj))


personality_schema = PersonalitySchemaSQLAutoWith(exclude=("id",))
upload_personality_schema = PersonalitySchemaSQLAutoWith(
    exclude=("id", "personality_id")
)
update_personality_schema = PersonalitySchemaSQLAutoWith(
    exclude=("id", "personality_id"), partial=True
)
personalities_schema = PersonalitySchemaSQLAutoWith(exclude=("id",), many=True)
