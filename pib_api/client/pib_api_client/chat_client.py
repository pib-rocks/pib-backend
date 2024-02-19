from urllib.request import Request
from typing import Any
from pib_api_client import send_request, URL_PREFIX
import json


CHAT_URL = URL_PREFIX + '/voice-assistant/chat/%s'
CHAT_MESSAGES_URL = URL_PREFIX + '/voice-assistant/chat/%s/messages'


def get_chat(chat_id: str) -> (bool, dict[str, Any]):
        request = Request(CHAT_URL % chat_id, method='GET')
        return send_request(request)

def create_chat_message(chat_id: str, message_content: str, is_user: bool) -> (bool, dict[str, Any]):
        data = json.dumps({
            "isUser": is_user,
            "content": message_content
        }).encode('UTF-8')
        request = Request(
                CHAT_MESSAGES_URL % chat_id, 
                method='POST',
                headers = { 'Content-Type': 'application/json' },
                data = data)
        return send_request(request)