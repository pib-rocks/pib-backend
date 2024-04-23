from urllib.request import Request
from typing import Any, Tuple
from pib_api_client import send_request, URL_PREFIX
import json

ASSISTANT_MODEL_URL = URL_PREFIX + 'assistant-model/%s'
PERSONALITY_URL = URL_PREFIX + '/voice-assistant/personality/%s'
CHAT_URL = URL_PREFIX + '/voice-assistant/chat/%s'
CHAT_MESSAGES_URL = URL_PREFIX + '/voice-assistant/chat/%s/messages'


class AssistantModel:
    def __init__(self, assistant_dto: dict[str, Any]):
        self.model_id = assistant_dto['model_id']
        self.api_name = assistant_dto['api_name']
        self.visual_name = assistant_dto['visual_name']
        self.has_image_support = assistant_dto['has_image_support']


class Personality:
    def __init__(self, personality_dto: dict[str, Any]):
        self.gender = personality_dto["gender"]
        self.language = "German"  # TODO: language should be stored as part of a personality -> personality_dto["language"]
        self.pause_threshold = personality_dto["pauseThreshold"]
        self.description = personality_dto.get("description")
        self.assistant_model = self._get_assistant_model(personality_dto["assistantModel"])

    def _get_assistant_model(self, assistant_id: int) -> AssistantModel:
        successful, model = get_assistant_model(assistant_id)
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


def get_assistant_model(assistant_model_id: int) -> Tuple[bool, AssistantModel]:
    request = Request(PERSONALITY_URL % assistant_model_id, method='GET')
    successful, assistant_model_dto = send_request(request)
    return successful, AssistantModel(assistant_model_dto)

def get_personality(personality_id: str) -> Tuple[bool, Personality]:
    request = Request(PERSONALITY_URL % personality_id, method='GET')
    successful, personality_dto = send_request(request)
    return successful, Personality(personality_dto)


def get_chat(chat_id: str) -> Tuple[bool, Chat]:
    request = Request(CHAT_URL % chat_id, method='GET')
    successful, chat_dto = send_request(request)
    return successful, Chat(chat_dto)


def get_personality_from_chat(chat_id: str) -> Tuple[bool, Personality]:
    successful, chat = get_chat(chat_id)
    if not successful: return False, None
    return get_personality(chat.personality_id)


def create_chat_message(chat_id: str, message_content: str, is_user: bool) -> Tuple[bool, ChatMessage]:
    data = json.dumps({
        "isUser": is_user,
        "content": message_content
    }).encode('UTF-8')
    request = Request(
        CHAT_MESSAGES_URL % chat_id,
        method='POST',
        headers={'Content-Type': 'application/json'},
        data=data)
    successful, chat_message_dto = send_request(request)
    return successful, ChatMessage(chat_message_dto)
