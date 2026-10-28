from urllib.request import Request
from typing import Any
from pib_api_client import send_request, URL_PREFIX
import json

DETECTED_ASSIGNMENTS_URL = URL_PREFIX + "/controller/detected-assignments"


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
