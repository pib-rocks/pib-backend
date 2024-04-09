from urllib.request import Request
from typing import Any, Tuple
from pib_api_client import send_request, URL_PREFIX

PERSONALITY_URL = URL_PREFIX + '/voice-assistant/personality/%s'


def get_personality(personality_id: str) -> (bool, dict[str, Any]):
    request = Request(PERSONALITY_URL % personality_id, method='GET')
    return send_request(request)


def get_assistant_model(assistant_id: int) -> tuple[str, bool]:
    request = Request(f"{URL_PREFIX}/assistant-model/{assistant_id}", method='GET')
    successful, response = send_request(request)
    if not successful:
        return "gpt-4", False
    return response["api_name"], response["has_image_support"]
