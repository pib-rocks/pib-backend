"""SmartHome hub API and hardware-variant coverage."""

from pathlib import Path

import pytest

from seed_profiles import PROFILES, get_profile
from service.system_property_service import ALLOWED_HARDWARE_VARIANTS

REPO_ROOT = Path(__file__).resolve().parents[2]
HA_CONFIG = REPO_ROOT / "setup" / "setup_files" / "homeassistant_configuration.yaml"

ALL_VARIANTS = (
    "pib4edu",
    "pib4advanced",
    "pib5edu",
    "pib5advanced",
    "pib5museum",
)


@pytest.mark.parametrize("variant", ALL_VARIANTS)
def test_all_variants_have_seed_profiles(variant: str):
    profile = get_profile(variant)
    assert profile.variant == variant
    assert variant in PROFILES


@pytest.mark.parametrize("variant", ALL_VARIANTS)
def test_smart_home_status_on_every_hardware_variant(app, monkeypatch, variant: str):
    monkeypatch.setenv("PIB_HARDWARE_VARIANT", variant)
    with app.test_client() as client:
        response = client.get("/smart-home/status")
    assert response.status_code == 200
    body = response.get_json()
    assert set(body["supportedVariants"]) == set(ALLOWED_HARDWARE_VARIANTS)
    assert body["homeAssistant"]["profile"] == "minimal-in-memory"
    assert body["homeAssistant"]["memoryCapMib"] == 384
    assert body["matterServer"]["memoryCapMib"] == 192


def test_hub_initialize_endpoint(app):
    with app.test_client() as client:
        response = client.post("/smart-home/initialize")
    assert response.status_code == 200
    assert "homeAssistant" in response.get_json()


def test_minimal_home_assistant_config_file():
    text = HA_CONFIG.read_text(encoding="utf-8")
    assert "default_config:" not in text
    assert "sqlite:///:memory:" in text


def test_matter_commission_valid_setup_code(app):
    with app.test_client() as client:
        response = client.post(
            "/smart-home/matter/commission",
            json={
                "pairingCode": "3497-011-2332",
                "friendlyName": "Desk Lamp",
                "room": "Office",
            },
        )
    assert response.status_code == 201
    body = response.get_json()
    assert body["commissioned"] is True
    assert body["device"]["friendlyName"] == "Desk Lamp"
    assert body["device"]["room"] == "Office"
    assert body["device"]["entityId"].startswith("light.matter_")


def test_matter_commission_valid_mt_qr_payload(app):
    with app.test_client() as client:
        response = client.post(
            "/smart-home/matter/commission",
            json={"pairingCode": "MT:ABC123DEF", "domain": "switch"},
        )
    assert response.status_code == 201
    assert response.get_json()["device"]["domain"] == "switch"


def test_matter_commission_rejects_invalid_code(app):
    with app.test_client() as client:
        response = client.post(
            "/smart-home/matter/commission",
            json={"pairingCode": "not-a-valid-code"},
        )
    assert response.status_code == 400
    assert "Invalid Matter pairing code" in response.get_json()["error"]


def _create_light(client, entity_id="light.living_room_lamp", **extra):
    payload = {
        "entityId": entity_id,
        "domain": "light",
        "friendlyName": extra.pop("friendlyName", "Living Room Lamp"),
        "room": extra.pop("room", "Living Room"),
        "exposeToVoice": extra.pop("exposeToVoice", True),
        **extra,
    }
    return client.post("/smart-home/devices", json=payload)


def test_device_crud_filter_and_actions(app):
    with app.test_client() as client:
        assert _create_light(client).status_code == 201

        listed = client.get("/smart-home/devices")
        assert listed.status_code == 200
        assert len(listed.get_json()["devices"]) == 1

        by_room = client.get(
            "/smart-home/devices", query_string={"room": "Living Room"}
        )
        assert len(by_room.get_json()["devices"]) == 1
        by_domain = client.get("/smart-home/devices", query_string={"domain": "light"})
        assert len(by_domain.get_json()["devices"]) == 1

        patch = client.patch(
            "/smart-home/devices/light.living_room_lamp",
            json={
                "friendlyName": "Main Lamp",
                "room": "Lounge",
                "exposeToVoice": False,
            },
        )
        assert patch.status_code == 200
        updated = patch.get_json()
        assert updated["friendlyName"] == "Main Lamp"
        assert updated["room"] == "Lounge"
        assert updated["exposeToVoice"] is False

        voice_only = client.get(
            "/smart-home/devices", query_string={"voiceOnly": "true"}
        )
        assert voice_only.get_json()["devices"] == []

        on = client.post(
            "/smart-home/devices/light.living_room_lamp/action",
            json={"action": "turn_on"},
        )
        assert on.status_code == 200
        assert on.get_json()["state"] == "on"
        assert on.get_json()["applied"] is True

        brightness = client.post(
            "/smart-home/devices/Main Lamp/action",
            json={"action": "set_brightness", "value": 180},
        )
        assert brightness.status_code == 200
        assert brightness.get_json()["attributes"]["brightness"] == 180

        off = client.post(
            "/smart-home/devices/light.living_room_lamp/action",
            json={"action": "turn_off"},
        )
        assert off.get_json()["state"] == "off"

        deleted = client.delete("/smart-home/devices/light.living_room_lamp")
        assert deleted.status_code == 200
        assert deleted.get_json()["deleted"] is True
        assert client.get("/smart-home/devices").get_json()["devices"] == []


def test_climate_set_temperature_action(app):
    with app.test_client() as client:
        client.post(
            "/smart-home/devices",
            json={
                "entityId": "climate.office",
                "domain": "climate",
                "friendlyName": "Office AC",
                "room": "Office",
            },
        )
        response = client.post(
            "/smart-home/devices/climate.office/action",
            json={"action": "set_temperature", "value": 21.5},
        )
    assert response.status_code == 200
    assert response.get_json()["state"] == "21.5"


def test_v1_smart_home_mirror(app):
    with app.test_client() as client:
        base = client.get("/smart-home/status").get_json()
        mirrored = client.get("/v1/smart-home/status").get_json()
    assert mirrored == base
