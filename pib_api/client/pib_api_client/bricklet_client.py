from urllib.request import Request
from typing import Any
from pib_api_client import send_request, URL_PREFIX

GET_ALL_BRICKLETS_URL = URL_PREFIX + "/bricklet"
GET_CONNECTED_BRICKLETS_URL = URL_PREFIX + "/bricklet/connected"
GET_ALL_CONTROLLERS_URL = URL_PREFIX + "/controller"


def get_all_bricklets() -> (bool, dict[str, Any]):
    """Transition helper for consumers that still need the legacy DTO."""
    request = Request(GET_ALL_BRICKLETS_URL, method="GET")
    return send_request(request)


def get_connected_bricklets() -> (bool, dict[str, Any]):
    """Devices brickd reports right now.

    The body is ``{"bricklets": [{"name", "uid", "port", "parentUid",
    "deviceIdentifier"}, ...]}``. Nginx exposes this as ``/api/bricklet/connected``;
    the Flask route, like ``/bricklet``, is mounted without the ``/api`` prefix.
    """
    request = Request(GET_CONNECTED_BRICKLETS_URL, method="GET")
    return send_request(request)


def get_all_controllers() -> (bool, dict[str, Any]):
    request = Request(GET_ALL_CONTROLLERS_URL, method="GET")
    return send_request(request)
