"""Unit tests for the SmartHome ROS bridge helpers and service dispatch."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
SMART_HOME_PKG = REPO_ROOT / "ros_packages" / "smart_home"
CLIENT_PATH = REPO_ROOT / "pib_api" / "client"

for path in (str(SMART_HOME_PKG), str(CLIENT_PATH)):
    if path not in sys.path:
        sys.path.insert(0, path)

from smart_home.smart_home_node import (  # noqa: E402
    device_dict_to_entity_fields,
    entity_fields_to_event,
    fill_entity_message,
    handle_call_service,
    handle_get_state,
    handle_list_entities,
    parse_action_value,
)


class FakeEntity:
    pass


class FakeBackend:
    def __init__(self, devices: list[dict]):
        self.devices = devices
        self.calls: list[tuple] = []

    def list_devices(self, room=None, domain=None, voice_only=False):
        self.calls.append(("list_devices", room, domain, voice_only))
        filtered = list(self.devices)
        if room:
            filtered = [d for d in filtered if d.get("room") == room]
        if domain:
            filtered = [d for d in filtered if d.get("domain") == domain]
        if voice_only:
            filtered = [d for d in filtered if d.get("exposeToVoice")]
        return True, {"devices": filtered}

    def get_device_state(self, target: str):
        self.calls.append(("get_device_state", target))
        lowered = target.casefold()
        for device in self.devices:
            if device.get("entityId") == target:
                return True, device
            if device.get("friendlyName", "").casefold() == lowered:
                return True, device
        return False, None

    def execute_action(self, target: str, action: str, value=None):
        self.calls.append(("execute_action", target, action, value))
        for device in self.devices:
            if device.get("entityId") == target or device.get("friendlyName") == target:
                updated = dict(device)
                if action == "turn_on":
                    updated["state"] = "on"
                elif value is not None:
                    updated["state"] = str(value)
                return True, updated
        return False, {"message": "not found"}


SAMPLE_DEVICE = {
    "entityId": "light.living_room",
    "domain": "light",
    "friendlyName": "Living Room",
    "room": "Living Room",
    "state": "off",
    "attributes": {"brightness": 10},
    "exposeToVoice": True,
    "exposeToPrograms": True,
    "online": True,
}


def test_device_dict_to_entity_fields_serializes_attributes():
    fields = device_dict_to_entity_fields(SAMPLE_DEVICE)
    assert fields["entity_id"] == "light.living_room"
    assert fields["friendly_name"] == "Living Room"
    assert json.loads(fields["attributes_json"]) == {"brightness": 10}
    assert fields["expose_to_voice"] is True
    assert fields["online"] is True


def test_fill_entity_message_sets_attributes():
    entity = FakeEntity()
    fill_entity_message(entity, SAMPLE_DEVICE)
    assert entity.entity_id == "light.living_room"
    assert entity.state == "off"


def test_parse_action_value_accepts_json_numbers_and_raw_strings():
    assert parse_action_value("") is None
    assert parse_action_value("21.5") == 21.5
    assert parse_action_value('"warm"') == "warm"
    assert parse_action_value("plain") == "plain"


def test_handle_list_entities_filters_room_and_domain():
    backend = FakeBackend(
        [
            SAMPLE_DEVICE,
            {
                **SAMPLE_DEVICE,
                "entityId": "switch.kitchen",
                "domain": "switch",
                "room": "Kitchen",
            },
        ]
    )
    ok, message, devices = handle_list_entities(backend, "Kitchen", "switch", False)
    assert ok is True
    assert message == "OK"
    assert len(devices) == 1
    assert devices[0]["entityId"] == "switch.kitchen"


def test_handle_get_state_resolves_friendly_name():
    backend = FakeBackend([SAMPLE_DEVICE])
    ok, message, device = handle_get_state(backend, "Living Room")
    assert ok is True
    assert device["entityId"] == "light.living_room"


def test_handle_call_service_publishes_event_and_returns_entity():
    backend = FakeBackend([SAMPLE_DEVICE])
    events: list[dict] = []

    ok, message, device = handle_call_service(
        backend,
        "light.living_room",
        "turn_on",
        "",
        publish_event=events.append,
    )

    assert ok is True
    assert message == "OK"
    assert device["state"] == "on"
    assert len(events) == 1
    assert events[0]["old_state"] == "off"
    assert events[0]["new_state"] == "on"
    assert events[0]["entity_id"] == "light.living_room"


def test_entity_fields_to_event_maps_states():
    event = entity_fields_to_event(SAMPLE_DEVICE, "off", "on")
    assert event["friendly_name"] == "Living Room"
    assert event["old_state"] == "off"
    assert event["new_state"] == "on"
