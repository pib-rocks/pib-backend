import json
from typing import Any
from urllib.request import Request

from pib_api_client import URL_PREFIX, send_request

SMART_HOME_URL = URL_PREFIX + "/smart-home"


def get_status() -> tuple[bool, dict[str, Any]]:
    request = Request(SMART_HOME_URL + "/status", method="GET")
    return send_request(request)


def list_devices(
    room: str | None = None,
    domain: str | None = None,
    voice_only: bool = False,
) -> tuple[bool, dict[str, Any]]:
    query = []
    if room is not None:
        query.append(f"room={room}")
    if domain is not None:
        query.append(f"domain={domain}")
    if voice_only:
        query.append("voiceOnly=true")
    suffix = "?" + "&".join(query) if query else ""
    request = Request(SMART_HOME_URL + "/devices" + suffix, method="GET")
    return send_request(request)


def get_device_state(target: str) -> tuple[bool, dict[str, Any]]:
    ok, payload = list_devices()
    if not ok or not payload:
        return ok, payload
    lowered = target.casefold()
    for device in payload.get("devices", []):
        entity_id = device.get("entityId", "")
        friendly = device.get("friendlyName", "")
        if entity_id == target or friendly == target:
            return True, device
        if lowered in entity_id.casefold() or lowered in friendly.casefold():
            return True, device
    return False, None


def execute_action(
    target: str, action: str, value: Any = None
) -> tuple[bool, dict[str, Any]]:
    body: dict[str, Any] = {"action": action}
    if value is not None:
        body["value"] = value
    request = Request(
        SMART_HOME_URL + f"/devices/{target}/action",
        method="POST",
        headers={"Content-Type": "application/json"},
        data=json.dumps(body).encode("utf-8"),
    )
    return send_request(request)


def commission_matter_device(
    pairing_code: str,
    *,
    friendly_name: str | None = None,
    room: str = "General",
    domain: str = "light",
    wifi_ssid: str | None = None,
    wifi_password: str | None = None,
) -> tuple[bool, dict[str, Any]]:
    payload: dict[str, Any] = {
        "pairingCode": pairing_code,
        "room": room,
        "domain": domain,
    }
    if friendly_name is not None:
        payload["friendlyName"] = friendly_name
    if wifi_ssid is not None:
        payload["wifiSsid"] = wifi_ssid
    if wifi_password is not None:
        payload["wifiPassword"] = wifi_password
    request = Request(
        SMART_HOME_URL + "/matter/commission",
        method="POST",
        headers={"Content-Type": "application/json"},
        data=json.dumps(payload).encode("utf-8"),
    )
    return send_request(request)
