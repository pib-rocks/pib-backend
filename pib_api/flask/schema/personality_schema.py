from __future__ import annotations

from marshmallow import ValidationError, fields, validate
from model.personality_model import (
    DEFAULT_GENDER,
    DEFAULT_MESSAGE_HISTORY,
    DEFAULT_PAUSE_THRESHOLD,
    Personality,
)
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
from pib_hermes_config.memory import (
    CHARACTER_LABEL,
    EXPERIENCE_LABEL,
    memory_size,
    read_memory,
)
from pib_hermes_config.voice_backends import live_voice_note, local_voice_applies
from provider_registry import has_capability
from schema.sql_auto_with_camel_case_schema import SQLAutoWithCamelCaseSchema
from service import provider_service, soul_service


class PersonalitySchemaSQLAutoWith(SQLAutoWithCamelCaseSchema):
    class Meta:
        model = Personality
        include_fk = True

    # A create needs the name only. The generated schema would make every
    # non-nullable column required, so the three without a server default
    # are declared here with the model's defaults. The ranges are the ones
    # Cerebra's dialog enforces, so the API rejects what the dialog rejects.
    gender = fields.String(required=False, load_default=DEFAULT_GENDER)
    pause_threshold = fields.Float(
        required=False,
        load_default=DEFAULT_PAUSE_THRESHOLD,
        validate=validate.Range(min=0.1, max=3.0),
    )
    message_history = fields.Integer(
        required=False,
        load_default=DEFAULT_MESSAGE_HISTORY,
        validate=validate.Range(min=0),
    )
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
    # 'default', or the decimal id of a provider row, as text. The catalogue
    # api_name is not a reference and is rejected with an unknown provider.
    provider_ref = fields.String(required=False, allow_none=True)
    channel = fields.String(
        required=False,
        validate=validate.OneOf([CHANNEL_SMART, CHANNEL_DIRECT]),
    )
    effective_channel = fields.Method("get_effective_channel", dump_only=True)
    smart_chats_enabled = fields.Method("get_smart_chats_enabled", dump_only=True)
    soul_path = fields.Method("get_soul_path", dump_only=True)
    # Character is description / SOUL.md. Experience is MEMORY.md. The two
    # labels stay distinct so an edit of a fact is not an edit of character.
    character_label = fields.Constant(CHARACTER_LABEL)
    experience_label = fields.Constant(EXPERIENCE_LABEL)
    memory_size = fields.Method("get_memory_size", dump_only=True)
    memory = fields.Method(
        serialize="get_memory_text",
        deserialize="load_memory_text",
        required=False,
        allow_none=True,
    )
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
    needs_new_model = fields.Method("get_needs_new_model", dump_only=True)

    def get_soul_path(self, obj: Personality) -> str:
        return soul_service.soul_path_for(obj.personality_id)

    def get_memory_size(self, obj: Personality) -> int:
        return memory_size(obj.personality_id)

    def get_memory_text(self, obj: Personality) -> str:
        return read_memory(obj.personality_id)

    def load_memory_text(self, value):
        if value is None:
            return ""
        if not isinstance(value, str):
            raise ValidationError("Memory must be text.")
        return value

    def get_effective_channel(self, obj: Personality) -> str:
        return effective_channel(obj.channel)

    def get_smart_chats_enabled(self, _obj: Personality) -> bool:
        return smart_chats_enabled()

    def _resolved_provider(self, obj: Personality):
        ref = getattr(obj, "provider_ref", None)
        if not ref:
            return None
        return provider_service.find_provider(str(ref))

    def get_needs_new_model(self, obj: Personality) -> bool:
        """True when the referenced model row is gone and settings must replace it.

        There is no status to read: a removed model has no row at all.
        """
        ref = getattr(obj, "provider_ref", None)
        if not ref:
            return False
        return self._resolved_provider(obj) is None

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
