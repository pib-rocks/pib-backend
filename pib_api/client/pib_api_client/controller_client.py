from urllib.request import Request
from typing import Any
from pib_api_client import send_request, URL_PREFIX
import json

DETECTED_ASSIGNMENTS_URL = URL_PREFIX + "/controller/detected-assignments"
ALL_CONTROLLERS_URL = URL_PREFIX + "/controller"


def get_all_controllers() -> (bool, dict[str, Any]):
    """Controllers incl. their address, which is the serial adapter identifier."""
    request = Request(ALL_CONTROLLERS_URL, method="GET")
    return send_request(request)


def report_detected_assignments(
    assignments: list[dict[str, Any]],
) -> (bool, dict[str, Any]):
    request = Request(
        DETECTED_ASSIGNMENTS_URL,
        method="POST",
        headers={"Content-Type": "application/json"},
        data=json.dumps({"assignments": assignments}).encode("UTF-8"),
    )
    return send_request(request)
