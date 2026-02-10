from typing import Any
from urllib.request import Request
from urllib.parse import quote

from pib_api_client import send_request, URL_PREFIX

MOTOR_POSITIONS_OF_POSE_URL = URL_PREFIX + "/pose/%s/motor-positions"
POSE_BY_NAME_URL = URL_PREFIX + "/pose/by-name/%s"


def get_motor_positions_of_pose(pose_id) -> tuple[bool, dict[str, Any] | None]:
    request = Request(MOTOR_POSITIONS_OF_POSE_URL % pose_id, method="GET")
    return send_request(request)


def get_pose_by_name(pose_name: str) -> tuple[bool, dict[str, Any] | None]:
    encoded_name = quote(pose_name, safe="")
    request = Request(POSE_BY_NAME_URL % encoded_name, method="GET")
    return send_request(request)
