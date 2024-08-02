from urllib.request import Request
from pib_api_client import send_request, URL_PREFIX
from typing import Any

MOTOR_POSITIONS_OF_POSE_URL = URL_PREFIX + "/pose/%s/motor-positions"

def get_motor_positions_of_pose(pose_id) -> (bool, dict[str, Any]):
    request = Request(MOTOR_POSITIONS_OF_POSE_URL % pose_id, method="GET")
    return send_request(request)