from urllib.request import Request
from typing import Any
from pib_api_client import send_request, URL_PREFIX

GET_ALL_BRICKLETS_URL = URL_PREFIX + "/bricklet"
GET_ALL_CONTROLLERS_URL = URL_PREFIX + "/controller"


def get_all_bricklets() -> (bool, dict[str, Any]):
    """Transition helper for consumers that still need the legacy DTO."""
    request = Request(GET_ALL_BRICKLETS_URL, method="GET")
    return send_request(request)


def get_all_controllers() -> (bool, dict[str, Any]):
    request = Request(GET_ALL_CONTROLLERS_URL, method="GET")
    return send_request(request)
