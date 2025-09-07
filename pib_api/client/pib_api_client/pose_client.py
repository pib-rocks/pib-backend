from typing import Any
from urllib.request import Request

from pib_api_client import send_request, URL_PREFIX

MOTOR_POSITIONS_OF_POSE_URL = URL_PREFIX + "/pose/%s/motor-positions"
POSES_URL = URL_PREFIX + "/pose"


def get_motor_positions_of_pose(pose_id) -> (bool, dict[str, Any]):
    request = Request(MOTOR_POSITIONS_OF_POSE_URL % pose_id, method="GET")
    return send_request(request)

def get_poses() -> (bool, dict[str, Any]):
    request = Request(POSES_URL, method="GET")
    return send_request(request)
