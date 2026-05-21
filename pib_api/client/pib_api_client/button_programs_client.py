from urllib.request import Request
from typing import Any
from pib_api_client import send_request, URL_PREFIX

GET_BUTTON_PROGRAMS_URL = URL_PREFIX + "/button-programs"


def get_button_programs() -> (bool, list[dict[str, Any]]):
    request = Request(GET_BUTTON_PROGRAMS_URL, method="GET")
    return send_request(request)
