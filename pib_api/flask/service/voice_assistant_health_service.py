"""Voice-assistant readiness for the system status UI (PR-2122).

Reports the same two facts the container HEALTHCHECK enforces: the chat stack
is answering on ROS and a provider key is resolvable for the active profile.
When the voice container is absent or Docker cannot run the probe, every field
stays ``unknown`` so the UI never paints a neutral gap as an outage.
"""

from __future__ import annotations

import json
import logging
import os
import re
from typing import Any, Dict, List, Optional, Tuple

from service.docker_admin_service import DOCKER_SOCK, _docker_request

LOGGER = logging.getLogger(__name__)

STATE_HEALTHY = "healthy"
STATE_UNHEALTHY = "unhealthy"
STATE_UNKNOWN = "unknown"
STATE_OK = "ok"
STATE_ATTENTION = "attention"
STATE_MISSING = "missing"

REASON_CONTAINER_ABSENT = "container_absent"
REASON_CHECK_UNAVAILABLE = "check_unavailable"
REASON_NO_KEY = "no_key"
REASON_CHAT_NODE = "chat_node"
REASON_CHAT_SERVICE = "chat_service"
REASON_KEY_CHECK = "key_check"

_VOICE_NAME_MARKERS = ("ros-voice-assistant",)

_REASON_PATTERN = re.compile(
    r"reason=(?P<reason>[A-Za-z0-9_]+)"
    r"(?:\s+provider=(?P<provider>[^\s]+))?"
    r"(?:\s+detail=(?P<detail>.+))?"
)


def _payload(
    *,
    state: str,
    node_state: str,
    provider_key_state: str,
    container_health: str = STATE_UNKNOWN,
    reason: Optional[str] = None,
    detail: str = "",
    provider: Optional[str] = None,
    node_reason: Optional[str] = None,
    key_reason: Optional[str] = None,
) -> dict:
    return {
        "state": state,
        "node": {
            "state": node_state,
            "reason": node_reason,
            "detail": detail if node_reason else "",
        },
        "providerKey": {
            "state": provider_key_state,
            "reason": key_reason,
            "detail": detail if key_reason else "",
            "provider": provider,
        },
        "containerHealth": container_health,
        "reason": reason,
        "detail": detail,
        "provider": provider,
    }


def _unknown(reason: str, detail: str = "") -> dict:
    return _payload(
        state=STATE_UNKNOWN,
        node_state=STATE_UNKNOWN,
        provider_key_state=STATE_UNKNOWN,
        reason=reason,
        detail=detail,
    )


def _find_voice_container(containers: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    for entry in containers:
        name = str(entry.get("name") or "")
        if any(marker in name for marker in _VOICE_NAME_MARKERS):
            return entry
    return None


def _list_containers() -> List[Dict[str, Any]]:
    if not os.path.exists(DOCKER_SOCK):
        return []
    try:
        status_code, body = _docker_request("GET", "/containers/json?all=true")
        if status_code != 200:
            return []
        raw = json.loads(body.decode("utf-8"))
    except Exception as error:  # noqa: BLE001
        LOGGER.info("Could not list containers for voice health: %s", error)
        return []

    result: List[Dict[str, Any]] = []
    for item in raw:
        names = item.get("Names") or []
        name = names[0].lstrip("/") if names else ""
        result.append(
            {
                "name": name,
                "state": item.get("State", "unknown"),
                "statusText": item.get("Status", ""),
            }
        )
    return result


def _inspect_container(name: str) -> Optional[Dict[str, Any]]:
    try:
        status_code, body = _docker_request("GET", f"/containers/{name}/json")
        if status_code != 200:
            return None
        return json.loads(body.decode("utf-8"))
    except Exception as error:  # noqa: BLE001
        LOGGER.info("Could not inspect voice container %s: %s", name, error)
        return None


def _parse_probe_output(output: str) -> Tuple[Optional[str], Optional[str], str]:
    text = (output or "").strip()
    if not text:
        return None, None, ""
    last_line = text.splitlines()[-1].strip()
    match = _REASON_PATTERN.search(last_line)
    if not match:
        return None, None, last_line
    return (
        match.group("reason"),
        match.group("provider"),
        (match.group("detail") or "").strip(),
    )


def _health_from_inspect(info: Dict[str, Any]) -> Tuple[str, str]:
    state = info.get("State") or {}
    health = state.get("Health")
    if not isinstance(health, dict):
        return STATE_UNKNOWN, ""

    status = str(health.get("Status") or STATE_UNKNOWN).strip().lower()
    log = health.get("Log")
    output = ""
    if isinstance(log, list) and log:
        last = log[-1]
        if isinstance(last, dict):
            output = str(last.get("Output") or "")
    return status or STATE_UNKNOWN, output


def _states_for_probe(
    docker_health: str, output: str
) -> Tuple[str, str, str, Optional[str], str, Optional[str], Optional[str]]:
    """Map Docker health + probe stderr into API fields.

    The aggregate ``state`` speaks the UI contract - ``ok``, ``attention`` or
    ``unknown`` - and is the only field the status chip reads:

    * ``ok``            probe passed
    * ``attention``     a real, actionable fault: no provider key for the route,
                        or the chat nodes are not up while the container runs
    * ``unknown``       we could not tell (absent container, unreadable inspect,
                        a failed key check) - never painted as a fault
    """
    reason, provider, detail = _parse_probe_output(output)

    if docker_health == STATE_HEALTHY:
        return (
            STATE_OK,
            STATE_HEALTHY,
            STATE_OK,
            provider,
            "",
            None,
            None,
        )

    if reason == REASON_NO_KEY:
        return (
            STATE_ATTENTION,
            STATE_HEALTHY,
            STATE_MISSING,
            provider,
            detail or "No provider key is stored for this route.",
            None,
            REASON_NO_KEY,
        )

    if reason in (REASON_CHAT_NODE, REASON_CHAT_SERVICE):
        return (
            STATE_ATTENTION,
            STATE_UNHEALTHY,
            STATE_UNKNOWN,
            provider,
            detail or reason,
            reason,
            None,
        )

    if reason == REASON_KEY_CHECK:
        return (
            STATE_UNKNOWN,
            STATE_HEALTHY,
            STATE_UNKNOWN,
            provider,
            detail or "The provider key check failed.",
            None,
            REASON_KEY_CHECK,
        )

    if docker_health == STATE_UNHEALTHY:
        return (
            STATE_ATTENTION,
            STATE_UNHEALTHY,
            STATE_UNKNOWN,
            provider,
            detail or "The voice health probe failed.",
            REASON_CHAT_NODE,
            None,
        )

    return STATE_UNKNOWN, STATE_UNKNOWN, STATE_UNKNOWN, provider, detail, None, None


def voice_assistant_status() -> dict:
    """Server-side voice readiness. Never raises."""
    containers = _list_containers()
    entry = _find_voice_container(containers)
    if entry is None:
        return _unknown(
            REASON_CONTAINER_ABSENT,
            "The voice-assistant container is not deployed on this host.",
        )

    name = entry["name"]
    state = str(entry.get("state") or "").lower()
    if state != "running":
        return _unknown(
            REASON_CONTAINER_ABSENT,
            f"The voice-assistant container is {state or 'not running'}.",
        )

    info = _inspect_container(name)
    if info is None:
        return _unknown(
            REASON_CHECK_UNAVAILABLE,
            "Could not inspect the voice-assistant container.",
        )

    docker_health, output = _health_from_inspect(info)
    if docker_health == STATE_UNKNOWN:
        status_text = str(entry.get("statusText") or "")
        if "healthy" in status_text.lower():
            docker_health = STATE_HEALTHY
        elif "unhealthy" in status_text.lower():
            docker_health = STATE_UNHEALTHY

    (
        aggregate,
        node_state,
        key_state,
        provider,
        detail,
        node_reason,
        key_reason,
    ) = _states_for_probe(docker_health, output)
    top_reason = key_reason or node_reason

    return _payload(
        state=aggregate,
        node_state=node_state,
        provider_key_state=key_state,
        container_health=docker_health,
        reason=top_reason,
        detail=detail,
        provider=provider,
        node_reason=node_reason,
        key_reason=key_reason,
    )
