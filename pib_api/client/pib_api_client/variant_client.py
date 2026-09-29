from typing import Any
from urllib.request import Request

from pib_api_client import URL_PREFIX, send_request

HARDWARE_VARIANT_URL = URL_PREFIX + "/system/hardware-variant"
HARDWARE_CAPABILITIES_URL = URL_PREFIX + "/system/hardware-capabilities"
SYSTEM_PROPERTIES_URL = URL_PREFIX + "/system/properties"


def get_hardware_variant() -> tuple[bool, dict[str, Any]]:
    request = Request(HARDWARE_VARIANT_URL, method="GET")
    return send_request(request)


def get_hardware_capabilities() -> tuple[bool, dict[str, Any]]:
    request = Request(HARDWARE_CAPABILITIES_URL, method="GET")
    return send_request(request)


def get_system_properties() -> tuple[bool, dict[str, Any]]:
    request = Request(SYSTEM_PROPERTIES_URL, method="GET")
    return send_request(request)
