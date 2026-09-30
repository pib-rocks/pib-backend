import json
from typing import Any, Optional, Tuple, List
from urllib.request import Request, urlopen

from pib_api_client import send_request, URL_PREFIX

ASSISTANT_MODEL_URL = URL_PREFIX + "/assistant-model/%s"
PROVIDER_DEFAULT_URL = URL_PREFIX + "/provider/default"
PERSONALITY_URL = URL_PREFIX + "/voice-assistant/personality/%s"
CHAT_URL = URL_PREFIX + "/voice-assistant/chat/%s"
CHAT_MESSAGES_URL = URL_PREFIX + "/voice-assistant/chat/%s/messages"


class AssistantModel:
    def __init__(self, assistant_dto: dict[str, Any]):
        self.model_id = assistant_dto["id"]
        self.api_name = assistant_dto["apiName"]
        self.visual_name = assistant_dto["visualName"]
        self.has_image_support = assistant_dto["hasImageSupport"]


class Personality:
    def __init__(self, personality_dto: dict[str, Any]):
        self.personality_id = personality_dto.get("personalityId")
        self.name = personality_dto.get("name") or ""
        self.soul_path = personality_dto.get("soulPath")
        self.gender = personality_dto["gender"]
        self.language = "German"  # TODO: language should be stored as part of a personality -> personality_dto["language"]
        self.pause_threshold = personality_dto["pauseThreshold"]
        raw_filler = personality_dto.get("thinkingFiller")
        if isinstance(raw_filler, str):
            raw_filler = raw_filler.strip()
            self.thinking_filler = raw_filler or None
        else:
            self.thinking_filler = None
        self.message_history = personality_dto["messageHistory"]
        self.description = personality_dto.get("description")
        self.stt_engine = personality_dto.get("sttEngine", "local_whisper")
        self.tts_engine = personality_dto.get("ttsEngine", "supertone")
        applies = personality_dto.get("localVoiceApplies", True)
        self.local_voice_applies = True if applies is None else bool(applies)
        self.live_voice_note = personality_dto.get("liveVoiceNote")
        self.provider_ref = personality_dto.get("providerRef")
        self.channel = personality_dto.get("channel") or "smart"
        # Absent means on. Only an explicit false disables tools and images.
        raw_tools = personality_dto.get("toolCalling", True)
        self.tool_calling = True if raw_tools is None else bool(raw_tools)
        self.voice_mode = personality_dto.get("voiceMode") or "live"
        self.live_model = personality_dto.get("liveModel")
        self.voice_start_mode = personality_dto.get("voiceStartMode") or "turn_based"
        raw_idle = personality_dto.get("liveIdleTimeout", 60)
        try:
            self.live_idle_timeout = int(raw_idle)
        except (TypeError, ValueError):
            self.live_idle_timeout = 60
        reported = personality_dto.get("effectiveChannel")
        self.effective_channel = (
            reported if reported in ("smart", "direct") else self.channel
        )
        self.assistant_model = self._resolve_assistant_model(personality_dto)

    def _resolve_assistant_model(
        self, personality_dto: dict[str, Any]
    ) -> AssistantModel:
        kind, model_id = model_endpoint_for(personality_dto)
        if kind == "default":
            successful, model = get_default_provider()
        else:
            successful, model = get_assistant_model(model_id)
        if not successful:
            raise Exception("Could not find the assistant model")
        return model


class Chat:

    def __init__(self, chat_dto: dict[str, Any]):
        self.chatId = chat_dto["chatId"]
        self.topic = chat_dto["topic"]
        self.personality_id = chat_dto["personalityId"]


class ChatMessage:

    def __init__(self, chat_message_dto: dict[str, Any]):
        self.message_id = chat_message_dto["messageId"]
        self.timestamp = chat_message_dto["timestamp"]
        self.is_user = chat_message_dto["isUser"]
        self.content = chat_message_dto["content"]


def model_endpoint_for(
    personality_dto: dict[str, Any],
) -> Tuple[str, Optional[int]]:
    """Decide which registry read resolves this personality.

    A stored providerRef of 'default' stays a pointer: the caller asks for the
    current default instead of an id copied onto the personality. An explicit
    reference is that provider's id.
    """
    provider_ref = personality_dto.get("providerRef")
    model_id = personality_dto.get("assistantModelId")
    if provider_ref == "default" or (provider_ref in (None, "") and model_id is None):
        return "default", None
    if provider_ref not in (None, "") and str(provider_ref).isdigit():
        return "id", int(provider_ref)
    return "id", int(model_id)


def get_assistant_model(assistant_model_id: int) -> Tuple[bool, AssistantModel]:
    request = Request(ASSISTANT_MODEL_URL % assistant_model_id, method="GET")
    successful, assistant_model_dto = send_request(request)
    if not successful or not isinstance(assistant_model_dto, dict):
        return False, None
    return True, AssistantModel(assistant_model_dto)


def get_default_provider() -> Tuple[bool, AssistantModel]:
    request = Request(PROVIDER_DEFAULT_URL, method="GET")
    successful, provider_dto = send_request(request)
    if not successful or not isinstance(provider_dto, dict):
        return False, None
    return True, AssistantModel(provider_dto)


def get_personality(personality_id: str) -> Tuple[bool, Personality]:
    request = Request(PERSONALITY_URL % personality_id, method="GET")
    successful, personality_dto = send_request(request)
    try:
        personality = Personality(personality_dto)
    except Exception:
        successful = False
        personality = None
    return successful, personality


def record_first_token_latency(chat_id: str, latency_ms: float) -> bool:
    """Store one measurement. A missing API must not stall the turn."""
    data = json.dumps({"latencyMs": latency_ms}).encode("utf-8")
    request = Request(
        CHAT_URL % chat_id + "/first-token-latency",
        method="PUT",
        headers={"Content-Type": "application/json"},
        data=data,
    )
    try:
        with urlopen(request, timeout=0.25) as response:
            response.read()
        return True
    except Exception:
        return False


def get_chat(chat_id: str) -> Tuple[bool, Chat]:
    request = Request(CHAT_URL % chat_id, method="GET")
    successful, chat_dto = send_request(request)
    return successful, Chat(chat_dto)


def get_personality_from_chat(chat_id: str) -> Tuple[bool, Personality]:
    successful, chat = get_chat(chat_id)
    if not successful:
        return False, None
    return get_personality(chat.personality_id)


def create_chat_message(
    chat_id: str, message_content: str, is_user: bool
) -> Tuple[bool, ChatMessage]:
    data = json.dumps({"isUser": is_user, "content": message_content}).encode("UTF-8")
    request = Request(
        CHAT_MESSAGES_URL % chat_id,
        method="POST",
        headers={"Content-Type": "application/json"},
        data=data,
    )
    successful, chat_message_dto = send_request(request)
    return successful, ChatMessage(chat_message_dto)


def update_chat_message(
    chat_id: str, message_content: str, is_user: bool, message_id: str
) -> Tuple[bool, ChatMessage]:
    data = json.dumps({"isUser": is_user, "content": message_content}).encode("UTF-8")
    request = Request(
        CHAT_MESSAGES_URL % chat_id + "/" + message_id,
        method="PUT",
        headers={"Content-Type": "application/json"},
        data=data,
    )
    successful, chat_message_dto = send_request(request)
    return successful, ChatMessage(chat_message_dto)


def get_all_chat_messages(chat_id: str) -> List[ChatMessage]:
    request = Request(CHAT_MESSAGES_URL % chat_id, method="GET")
    successful, chat_messages_dto = send_request(request)
    if not successful:
        return successful, None
    chat_message_dtos = chat_messages_dto["messages"]
    chat_messages = [
        ChatMessage(chat_message_dto) for chat_message_dto in chat_message_dtos
    ]
    return successful, chat_messages


def get_chat_history(chat_id: str, history_length: int) -> List[ChatMessage]:
    request = Request(
        CHAT_MESSAGES_URL % chat_id + f"/{history_length}",
        method="GET",
    )
    successful, chat_messages_dto = send_request(request)
    if not successful:
        return successful, None
    chat_message_dtos = chat_messages_dto["messages"]
    chat_messages = [
        ChatMessage(chat_message_dto) for chat_message_dto in chat_message_dtos
    ]
    return successful, chat_messages
