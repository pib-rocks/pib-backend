from typing import Any
from urllib.request import Request

from pib_api_client import URL_PREFIX, send_request

MOTION_MODEL_URL = URL_PREFIX + "/system/motion-model"


def get_motion_model() -> tuple[bool, dict[str, Any] | None]:
    request = Request(MOTION_MODEL_URL, method="GET")
    return send_request(request)
