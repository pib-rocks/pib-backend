"""Bridge pib SmartHome REST API to ROS 2 topics and services."""

from __future__ import annotations

import json
from typing import Any, Callable, Protocol

from pib_api_client import smart_home_client


class SmartHomeBackend(Protocol):
    def list_devices(
        self,
        room: str | None = None,
        domain: str | None = None,
        voice_only: bool = False,
    ) -> tuple[bool, dict[str, Any] | None]: ...

    def get_device_state(self, target: str) -> tuple[bool, dict[str, Any] | None]: ...

    def execute_action(
        self, target: str, action: str, value: Any = None
    ) -> tuple[bool, dict[str, Any] | None]: ...


def _optional_filter(value: str) -> str | None:
    stripped = (value or "").strip()
    return stripped if stripped else None


def device_dict_to_entity_fields(device: dict[str, Any]) -> dict[str, Any]:
    """Map a Flask SmartHome device dict to SmartHomeEntity field values."""

    attributes = device.get("attributes", {})
    if isinstance(attributes, str):
        attributes_json = attributes
    else:
        attributes_json = json.dumps(attributes or {})
    return {
        "entity_id": device.get("entityId", ""),
        "domain": device.get("domain", ""),
        "friendly_name": device.get("friendlyName", ""),
        "room": device.get("room", ""),
        "state": device.get("state", ""),
        "attributes_json": attributes_json,
        "expose_to_voice": bool(device.get("exposeToVoice", False)),
        "expose_to_programs": bool(device.get("exposeToPrograms", False)),
        "online": bool(device.get("online", False)),
    }


def entity_fields_to_event(
    device: dict[str, Any], old_state: str, new_state: str
) -> dict[str, Any]:
    fields = device_dict_to_entity_fields(device)
    return {
        "entity_id": fields["entity_id"],
        "domain": fields["domain"],
        "friendly_name": fields["friendly_name"],
        "room": fields["room"],
        "old_state": old_state,
        "new_state": new_state,
        "attributes_json": fields["attributes_json"],
    }


def parse_action_value(value_json: str) -> Any:
    text = (value_json or "").strip()
    if not text:
        return None
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        return value_json


def fill_entity_message(entity: Any, device: dict[str, Any]) -> None:
    for key, value in device_dict_to_entity_fields(device).items():
        setattr(entity, key, value)


def handle_list_entities(
    backend: SmartHomeBackend,
    room: str,
    domain: str,
    voice_only: bool,
) -> tuple[bool, str, list[dict[str, Any]]]:
    ok, payload = backend.list_devices(
        room=_optional_filter(room),
        domain=_optional_filter(domain),
        voice_only=voice_only,
    )
    if not ok or not payload:
        message = "SmartHome device list unavailable"
        if isinstance(payload, dict) and payload.get("message"):
            message = str(payload["message"])
        return False, message, []
    devices = list(payload.get("devices", []))
    return True, "OK", devices


def handle_get_state(
    backend: SmartHomeBackend, target: str
) -> tuple[bool, str, dict[str, Any] | None]:
    stripped = (target or "").strip()
    if not stripped:
        return False, "target must not be empty", None
    ok, device = backend.get_device_state(stripped)
    if not ok or not device:
        return False, f"SmartHome device {stripped!r} not found", None
    return True, "OK", device


def handle_call_service(
    backend: SmartHomeBackend,
    target: str,
    action: str,
    value_json: str,
    publish_event: Callable[[dict[str, Any]], None] | None = None,
) -> tuple[bool, str, dict[str, Any] | None]:
    stripped_target = (target or "").strip()
    stripped_action = (action or "").strip()
    if not stripped_target:
        return False, "target must not be empty", None
    if not stripped_action:
        return False, "action must not be empty", None

    value = parse_action_value(value_json)
    ok_before, previous = backend.get_device_state(stripped_target)
    old_state = previous.get("state", "") if ok_before and previous else ""

    ok, result = backend.execute_action(stripped_target, stripped_action, value)
    if not ok or not result:
        message = f"SmartHome action {stripped_action!r} failed for {stripped_target!r}"
        if isinstance(result, dict) and result.get("message"):
            message = str(result["message"])
        return False, message, None

    new_state = str(result.get("state", old_state))
    if publish_event is not None:
        publish_event(entity_fields_to_event(result, old_state, new_state))
    return True, "OK", result


class SmartHomeNode:
    """ROS 2 node; use ``create_smart_home_node()`` to instantiate with rclpy."""

    def __init__(self, node: Any, backend: SmartHomeBackend | None = None):
        from datatypes.msg import SmartHomeEntity, SmartHomeEvent
        from datatypes.srv import (
            CallSmartHomeService,
            GetSmartHomeState,
            ListSmartHomeEntities,
        )

        self._node = node
        self._backend: SmartHomeBackend = backend or smart_home_client
        self._SmartHomeEntity = SmartHomeEntity
        self._event_publisher = node.create_publisher(
            SmartHomeEvent, "/smart_home/events", 10
        )

        node.create_service(
            ListSmartHomeEntities,
            "/smart_home/list_entities",
            self._on_list_entities,
        )
        node.create_service(
            GetSmartHomeState,
            "/smart_home/get_state",
            self._on_get_state,
        )
        node.create_service(
            CallSmartHomeService,
            "/smart_home/call_service",
            self._on_call_service,
        )

    def _publish_event(self, event_fields: dict[str, Any]) -> None:
        from datatypes.msg import SmartHomeEvent

        message = SmartHomeEvent()
        for key, value in event_fields.items():
            setattr(message, key, value)
        self._event_publisher.publish(message)

    def _on_list_entities(self, request, response):
        ok, message, devices = handle_list_entities(
            self._backend,
            request.room,
            request.domain,
            request.voice_only,
        )
        response.successful = ok
        response.message = message
        response.entities = []
        if ok:
            for device in devices:
                entity = self._SmartHomeEntity()
                fill_entity_message(entity, device)
                response.entities.append(entity)
        return response

    def _on_get_state(self, request, response):
        ok, message, device = handle_get_state(self._backend, request.target)
        response.successful = ok
        response.message = message
        if ok and device is not None:
            fill_entity_message(response.entity, device)
        return response

    def _on_call_service(self, request, response):
        ok, message, device = handle_call_service(
            self._backend,
            request.target,
            request.action,
            request.value_json,
            publish_event=self._publish_event,
        )
        response.successful = ok
        response.message = message
        if ok and device is not None:
            fill_entity_message(response.entity, device)
        return response


def create_smart_home_node(backend: SmartHomeBackend | None = None):
    from rclpy.node import Node

    class _RosSmartHomeNode(Node):
        def __init__(self):
            super().__init__("smart_home_node")
            self._bridge = SmartHomeNode(self, backend=backend)

    return _RosSmartHomeNode()


def main(args: list[str] | None = None) -> None:
    import rclpy

    rclpy.init(args=args)
    node = create_smart_home_node()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
