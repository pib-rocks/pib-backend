"""Home Assistant and Matter hub integration for pib SmartHome."""

from __future__ import annotations

import base64
import json
import logging
import os
import re
import secrets
import socket
import struct
from pathlib import Path
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.parse import urljoin, urlparse
from urllib.request import Request, urlopen

from sqlalchemy.exc import NoResultFound

from app.app import db
from model.smart_home_device_model import SmartHomeDevice
from service import key_store_service
from service.system_property_service import ALLOWED_HARDWARE_VARIANTS

logger = logging.getLogger(__name__)

HOMEASSISTANT_URL = os.environ.get(
    "HOMEASSISTANT_URL", "http://homeassistant:8123"
).rstrip("/")
MATTER_SERVER_URL = os.environ.get(
    "MATTER_SERVER_URL", "ws://host.docker.internal:5580/ws"
)
HOMEASSISTANT_TOKEN_PATH = Path(
    os.environ.get(
        "PIB_HOMEASSISTANT_TOKEN_PATH",
        "/home/pib/app/secrets/homeassistant_token.json",
    )
)
HOMEASSISTANT_TOKEN_KEY_STORE_ID = "homeassistant.access_token"

SUPPORTED_DOMAINS = frozenset(
    {
        "light",
        "switch",
        "sensor",
        "binary_sensor",
        "climate",
        "cover",
        "lock",
        "scene",
    }
)

HA_PROBE_TIMEOUT = 1.5
MATTER_PROBE_TIMEOUT = 1.0

SUPPORTED_ACTIONS = frozenset(
    {
        "turn_on",
        "turn_off",
        "toggle",
        "set_brightness",
        "set_color_rgb",
        "set_temperature",
        "set_cover_position",
        "lock",
        "unlock",
        "activate",
    }
)

_MATTER_SETUP_CODE_RE = re.compile(r"^\d{11}$|^\d{21}$")


class SmartHomeValidationError(ValueError):
    """A SmartHome request failed validation."""


def _ensure_table() -> None:
    SmartHomeDevice.__table__.create(bind=db.engine, checkfirst=True)


def _read_token_file() -> str | None:
    try:
        raw = HOMEASSISTANT_TOKEN_PATH.read_text(encoding="utf-8")
        document = json.loads(raw)
    except (OSError, json.JSONDecodeError):
        return None
    token = document.get("access_token") or document.get("token")
    if isinstance(token, str) and token.strip():
        return token.strip()
    return None


def _write_token_file(token: str) -> None:
    HOMEASSISTANT_TOKEN_PATH.parent.mkdir(parents=True, exist_ok=True)
    payload = {"access_token": token}
    HOMEASSISTANT_TOKEN_PATH.write_text(
        json.dumps(payload, indent=2) + "\n", encoding="utf-8"
    )


def _get_ha_token() -> str | None:
    try:
        secrets_map = key_store_service.unlocked_credentials()
        token = secrets_map.get(HOMEASSISTANT_TOKEN_KEY_STORE_ID)
        if isinstance(token, str) and token.strip():
            return token.strip()
    except Exception:
        pass
    return _read_token_file()


def _save_ha_token(token: str) -> None:
    _write_token_file(token)


def _ha_request(
    method: str,
    path: str,
    *,
    token: str | None = None,
    json_body: dict | None = None,
    timeout: float = HA_PROBE_TIMEOUT,
    auth_required: bool = True,
) -> tuple[int, Any]:
    url = urljoin(HOMEASSISTANT_URL + "/", path.lstrip("/"))
    headers = {"Content-Type": "application/json"}
    if auth_required and token:
        headers["Authorization"] = f"Bearer {token}"
    data = None
    if json_body is not None:
        data = json.dumps(json_body).encode("utf-8")
    request = Request(url, data=data, headers=headers, method=method)
    try:
        with urlopen(request, timeout=timeout) as response:
            body = response.read().decode("utf-8")
            if not body:
                return response.status, None
            return response.status, json.loads(body)
    except HTTPError as error:
        body = error.read().decode("utf-8", errors="replace")
        try:
            parsed = json.loads(body) if body else None
        except json.JSONDecodeError:
            parsed = body
        return error.code, parsed
    except (URLError, TimeoutError, OSError, json.JSONDecodeError) as exc:
        logger.debug("Home Assistant request failed: %s", exc)
        return 0, None


def _probe_home_assistant() -> tuple[bool, bool]:
    """Return (online, onboarding_complete)."""
    status, onboarding = _ha_request(
        "GET", "/api/onboarding", auth_required=False, timeout=HA_PROBE_TIMEOUT
    )
    if status == 200 and isinstance(onboarding, list):
        onboarding_complete = all(
            step.get("done") for step in onboarding if isinstance(step, dict)
        )
        return True, onboarding_complete

    token = _get_ha_token()
    if token:
        api_status, _payload = _ha_request("GET", "/api/", token=token)
        if api_status == 200:
            return True, True
    return False, False


def _matter_host_port() -> tuple[str, int]:
    parsed = urlparse(MATTER_SERVER_URL)
    host = parsed.hostname or "localhost"
    port = parsed.port or 5580
    return host, port


def _probe_matter_server() -> bool:
    host, port = _matter_host_port()
    try:
        with socket.create_connection((host, port), timeout=MATTER_PROBE_TIMEOUT):
            return True
    except OSError:
        return False


def _serialize_device(device: SmartHomeDevice, *, ha_online: bool) -> dict:
    return {
        "entityId": device.entity_id,
        "domain": device.domain,
        "friendlyName": device.friendly_name,
        "room": device.room,
        "exposeToVoice": device.expose_to_voice,
        "exposeToPrograms": device.expose_to_programs,
        "state": device.cached_state,
        "attributes": device.attributes,
        "online": ha_online and _get_ha_token() is not None,
    }


def get_hub_status() -> dict:
    _ensure_table()
    ha_online, onboarding_complete = _probe_home_assistant()
    matter_online = _probe_matter_server()
    device_count = SmartHomeDevice.query.count()
    voice_exposed_count = SmartHomeDevice.query.filter_by(expose_to_voice=True).count()
    return {
        "homeAssistant": {
            "online": ha_online,
            "url": HOMEASSISTANT_URL,
            "onboardingComplete": onboarding_complete,
            "profile": "minimal-in-memory",
            "memoryCapMib": 384,
        },
        "matterServer": {
            "online": matter_online,
            "url": MATTER_SERVER_URL,
            "memoryCapMib": 192,
        },
        "deviceCount": device_count,
        "voiceExposedCount": voice_exposed_count,
        "supportedVariants": list(ALLOWED_HARDWARE_VARIANTS),
    }


def initialize_hub() -> dict:
    _ensure_table()
    ha_online, onboarding_complete = _probe_home_assistant()
    if not ha_online:
        return get_hub_status()

    if not onboarding_complete:
        password = secrets.token_urlsafe(16)
        status, body = _ha_request(
            "POST",
            "/api/onboarding/users",
            auth_required=False,
            json_body={
                "client_id": "pib-backend",
                "name": "pib",
                "username": "pib",
                "password": password,
                "language": "en",
            },
            timeout=5.0,
        )
        if status in (200, 201) and isinstance(body, dict):
            auth_code = body.get("auth_code")
            if auth_code:
                token_status, token_body = _ha_request(
                    "POST",
                    "/auth/token",
                    auth_required=False,
                    json_body={
                        "grant_type": "authorization_code",
                        "code": auth_code,
                        "client_id": "pib-backend",
                    },
                    timeout=5.0,
                )
                if token_status == 200 and isinstance(token_body, dict):
                    access_token = token_body.get("access_token")
                    if isinstance(access_token, str) and access_token:
                        _save_ha_token(access_token)

    return get_hub_status()


def _sync_from_home_assistant() -> None:
    token = _get_ha_token()
    if not token:
        return
    ha_online, _ = _probe_home_assistant()
    if not ha_online:
        return
    status, states = _ha_request("GET", "/api/states", token=token, timeout=5.0)
    if status != 200 or not isinstance(states, list):
        return

    for entry in states:
        if not isinstance(entry, dict):
            continue
        entity_id = entry.get("entity_id")
        if not isinstance(entity_id, str) or "." not in entity_id:
            continue
        domain = entity_id.split(".", 1)[0]
        if domain not in SUPPORTED_DOMAINS:
            continue
        state = str(entry.get("state", "unknown"))
        attributes = entry.get("attributes") or {}
        friendly = attributes.get("friendly_name") or entity_id
        if not isinstance(friendly, str):
            friendly = entity_id

        device = SmartHomeDevice.query.filter_by(entity_id=entity_id).one_or_none()
        if device is None:
            device = SmartHomeDevice(
                entity_id=entity_id,
                domain=domain,
                friendly_name=friendly,
            )
            db.session.add(device)
        device.cached_state = state
        merged = dict(device.attributes)
        if isinstance(attributes, dict):
            merged.update(attributes)
        device.attributes = merged
        if device.friendly_name == entity_id and friendly != entity_id:
            device.friendly_name = friendly


def list_devices(
    room: str | None = None,
    domain: str | None = None,
    voice_only: bool = False,
) -> list[dict]:
    _ensure_table()
    _sync_from_home_assistant()
    ha_online, _ = _probe_home_assistant()

    query = SmartHomeDevice.query
    if room is not None:
        query = query.filter_by(room=room)
    if domain is not None:
        query = query.filter_by(domain=domain)
    if voice_only:
        query = query.filter_by(expose_to_voice=True)

    devices = query.order_by(SmartHomeDevice.entity_id).all()
    return [_serialize_device(device, ha_online=ha_online) for device in devices]


def register_or_update_device(entity_id: str, payload: dict) -> dict:
    _ensure_table()
    ha_online, _ = _probe_home_assistant()

    device = SmartHomeDevice.query.filter_by(entity_id=entity_id).one_or_none()
    if device is None:
        domain = payload.get("domain")
        if not domain or domain not in SUPPORTED_DOMAINS:
            raise SmartHomeValidationError(
                f"domain must be one of: {', '.join(sorted(SUPPORTED_DOMAINS))}"
            )
        friendly_name = payload.get("friendlyName") or entity_id
        device = SmartHomeDevice(
            entity_id=entity_id,
            domain=domain,
            friendly_name=friendly_name,
        )
        db.session.add(device)

    if "friendlyName" in payload and payload["friendlyName"]:
        device.friendly_name = payload["friendlyName"]
    if "room" in payload and payload["room"]:
        device.room = payload["room"]
    if "exposeToVoice" in payload:
        device.expose_to_voice = bool(payload["exposeToVoice"])
    if "exposeToPrograms" in payload:
        device.expose_to_programs = bool(payload["exposeToPrograms"])
    if "state" in payload and payload["state"] is not None:
        device.cached_state = str(payload["state"])
    if "attributes" in payload and isinstance(payload["attributes"], dict):
        merged = dict(device.attributes)
        merged.update(payload["attributes"])
        device.attributes = merged

    db.session.flush()
    return _serialize_device(device, ha_online=ha_online)


def delete_device(entity_id: str) -> None:
    _ensure_table()
    device = SmartHomeDevice.query.filter_by(entity_id=entity_id).one()
    db.session.delete(device)


def _resolve_device(target: str) -> SmartHomeDevice:
    device = SmartHomeDevice.query.filter_by(entity_id=target).one_or_none()
    if device is not None:
        return device

    lowered = target.casefold()
    for candidate in SmartHomeDevice.query.all():
        if candidate.friendly_name.casefold() == lowered:
            return candidate

    matches = [
        item
        for item in SmartHomeDevice.query.all()
        if lowered in item.friendly_name.casefold()
        or lowered in item.entity_id.casefold()
        or lowered in f"{item.room} {item.friendly_name}".casefold()
    ]
    if len(matches) == 1:
        return matches[0]
    if not matches:
        raise NoResultFound()
    raise SmartHomeValidationError(
        "Multiple devices match the target; use entityId for an unambiguous match."
    )


def _apply_local_action(
    device: SmartHomeDevice, action: str, value: Any, extra: dict | None
) -> None:
    attributes = dict(device.attributes)
    state = device.cached_state

    if action == "turn_on":
        state = "on"
    elif action == "turn_off":
        state = "off"
    elif action == "toggle":
        state = "off" if state == "on" else "on"
    elif action == "set_brightness":
        attributes["brightness"] = value
        state = "on"
    elif action == "set_color_rgb":
        attributes["rgb_color"] = value
        state = "on"
    elif action == "set_temperature":
        attributes["temperature"] = value
        if device.domain == "climate":
            state = str(value)
    elif action == "set_cover_position":
        attributes["current_position"] = value
        state = str(value)
    elif action == "lock":
        state = "locked"
    elif action == "unlock":
        state = "unlocked"
    elif action == "activate":
        state = "on"

    device.cached_state = state
    device.attributes = attributes


def _ha_service_for_action(
    domain: str, action: str, value: Any, extra: dict | None
) -> tuple[str, dict]:
    if action in {"turn_on", "turn_off", "toggle"}:
        return action, {}
    if action == "set_brightness":
        return "turn_on", {"brightness": value}
    if action == "set_color_rgb":
        return "turn_on", {"rgb_color": value}
    if action == "set_temperature":
        return "set_temperature", {"temperature": value}
    if action == "set_cover_position":
        return "set_cover_position", {"position": value}
    if action == "lock":
        return "lock", {}
    if action == "unlock":
        return "unlock", {}
    if action == "activate":
        return "turn_on", {}
    raise SmartHomeValidationError(
        f"Unsupported action {action!r} for domain {domain!r}"
    )


def _forward_to_home_assistant(
    device: SmartHomeDevice, action: str, value: Any, extra: dict | None
) -> None:
    token = _get_ha_token()
    if not token:
        return
    ha_online, _ = _probe_home_assistant()
    if not ha_online:
        return

    ha_service, service_data = _ha_service_for_action(
        device.domain, action, value, extra
    )
    payload = {"entity_id": device.entity_id, **(extra or {}), **service_data}
    service_domain = device.domain
    if action == "activate" and device.domain == "scene":
        service_domain = "scene"
    elif action in {"lock", "unlock"}:
        service_domain = "lock"

    _ha_request(
        "POST",
        f"/api/services/{service_domain}/{ha_service}",
        token=token,
        json_body=payload,
        timeout=5.0,
    )


def execute_device_action(
    target: str,
    action: str,
    value: Any = None,
    extra: dict | None = None,
) -> dict:
    _ensure_table()
    if action not in SUPPORTED_ACTIONS:
        raise SmartHomeValidationError(
            f"Invalid action {action!r}. Supported: {', '.join(sorted(SUPPORTED_ACTIONS))}"
        )

    device = _resolve_device(target)
    _apply_local_action(device, action, value, extra)
    _forward_to_home_assistant(device, action, value, extra)
    ha_online, _ = _probe_home_assistant()
    result = _serialize_device(device, ha_online=ha_online)
    result["action"] = action
    result["applied"] = True
    return result


def _normalize_matter_code(pairing_code: str) -> str:
    stripped = pairing_code.strip()
    if stripped.upper().startswith("MT:"):
        return stripped
    digits = re.sub(r"[\s\-]", "", stripped)
    if _MATTER_SETUP_CODE_RE.match(digits):
        return digits
    raise SmartHomeValidationError(
        "Invalid Matter pairing code. Expected an 11/21-digit setup code or MT:... QR payload."
    )


def _slugify(name: str) -> str:
    slug = re.sub(r"[^a-z0-9]+", "_", name.casefold()).strip("_")
    return slug or "device"


def _ws_send_command(message: dict) -> dict | None:
    parsed = urlparse(MATTER_SERVER_URL)
    host = parsed.hostname or "localhost"
    port = parsed.port or 5580
    path = parsed.path or "/ws"
    key = base64.b64encode(secrets.token_bytes(16)).decode("ascii")
    handshake = (
        f"GET {path} HTTP/1.1\r\n"
        f"Host: {host}:{port}\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        f"Sec-WebSocket-Key: {key}\r\n"
        "Sec-WebSocket-Version: 13\r\n"
        "\r\n"
    ).encode("ascii")

    try:
        with socket.create_connection(
            (host, port), timeout=MATTER_PROBE_TIMEOUT
        ) as sock:
            sock.sendall(handshake)
            response = b""
            while b"\r\n\r\n" not in response:
                chunk = sock.recv(4096)
                if not chunk:
                    break
                response += chunk
            if b"101" not in response.split(b"\r\n", 1)[0]:
                return None

            payload = json.dumps(message).encode("utf-8")
            mask = secrets.token_bytes(4)
            frame = bytearray()
            frame.append(0x81)
            length = len(payload)
            frame.append(0x80 | length)
            frame.extend(mask)
            frame.extend(bytes(b ^ mask[i % 4] for i, b in enumerate(payload)))
            sock.sendall(frame)

            header = sock.recv(2)
            if len(header) < 2:
                return None
            payload_len = header[1] & 0x7F
            if payload_len == 126:
                payload_len = struct.unpack("!H", sock.recv(2))[0]
            elif payload_len == 127:
                payload_len = struct.unpack("!Q", sock.recv(8))[0]
            masked = bool(header[1] & 0x80)
            if masked:
                mask_key = sock.recv(4)
            body = sock.recv(payload_len)
            if masked:
                body = bytes(b ^ mask_key[i % 4] for i, b in enumerate(body))
            return json.loads(body.decode("utf-8"))
    except (OSError, json.JSONDecodeError, struct.error) as exc:
        logger.debug("Matter WebSocket command failed: %s", exc)
        return None


def commission_matter_device(
    pairing_code: str,
    room: str = "General",
    friendly_name: str | None = None,
    domain: str = "light",
    wifi_ssid: str | None = None,
    wifi_password: str | None = None,
) -> dict:
    _ensure_table()
    normalized = _normalize_matter_code(pairing_code)
    if domain not in SUPPORTED_DOMAINS:
        raise SmartHomeValidationError(
            f"domain must be one of: {', '.join(sorted(SUPPORTED_DOMAINS))}"
        )

    label = friendly_name or f"Matter {domain}"
    if _probe_matter_server():
        command = {
            "message_id": secrets.token_hex(8),
            "command": "commission_with_code",
            "args": {"code": normalized},
        }
        if wifi_ssid:
            command["args"]["network_only"] = True
            command["args"]["wifi_ssid"] = wifi_ssid
            if wifi_password:
                command["args"]["wifi_password"] = wifi_password
        _ws_send_command(command)

    slug = _slugify(label)
    entity_id = f"{domain}.matter_{slug}"
    suffix = 2
    while SmartHomeDevice.query.filter_by(entity_id=entity_id).one_or_none():
        entity_id = f"{domain}.matter_{slug}_{suffix}"
        suffix += 1

    initial_state = "21.0" if domain in {"sensor", "climate"} else "off"
    device = register_or_update_device(
        entity_id,
        {
            "domain": domain,
            "friendlyName": label,
            "room": room,
            "exposeToVoice": True,
            "exposeToPrograms": True,
            "state": initial_state,
            "attributes": {"source": "matter", "pairingCodeHint": normalized[:4] + "…"},
        },
    )
    return {"commissioned": True, "device": device}


def get_device_state(target: str) -> dict:
    _ensure_table()
    ha_online, _ = _probe_home_assistant()
    device = _resolve_device(target)
    return _serialize_device(device, ha_online=ha_online)
