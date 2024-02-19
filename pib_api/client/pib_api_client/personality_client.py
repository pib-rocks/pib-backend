from urllib.request import Request
from typing import Any
from pib_api_client import send_request, URL_PREFIX


PERSONALITY_URL = URL_PREFIX + '/voice-assistant/personality/%s'


def get_personality(personality_id: str) -> (bool, dict[str, Any]):
        request = Request(PERSONALITY_URL % personality_id, method='GET')
        return send_request(request)