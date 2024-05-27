from urllib.request import Request
from typing import Any, Tuple
from pib_api_client import send_request, URL_PREFIX
import json


PERSONALITY_URL = URL_PREFIX + "/voice-assistant/personality/%s"
CHAT_URL = URL_PREFIX + "/voice-assistant/chat/%s"
CHAT_MESSAGES_URL = URL_PREFIX + "/voice-assistant/chat/%s/messages"


class Personality:

    def __init__(self, personality_dto: dict[str, Any]):
        self.gender = personality_dto["gender"]
        self.language = "German"  # TODO: language should be stored as part of a personality -> personality_dto["language"]
        self.pause_threshold = personality_dto["pauseThreshold"]
        self.description = personality_dto.get("description")


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


def get_personality(personality_id: str) -> Tuple[bool, Personality]:
    request = Request(PERSONALITY_URL % personality_id, method="GET")
    successful, personality_dto = send_request(request)
    return successful, Personality(personality_dto)


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
