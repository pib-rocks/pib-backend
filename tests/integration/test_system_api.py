import logging
from urllib.parse import urlsplit

import pytest

from app.app import db
from model.controller_model import (
    FEETECH_ST_SERIAL,
    ROBSTRIDE_CAN,
    TINKERFORGE_BRICKLET,
)
from model.system_property_model import SystemProperty
from service.system_property_service import (
    ALLOWED_HARDWARE_VARIANTS,
    HARDWARE_VARIANT_KEY,
)


def test_hardware_variant_shape_and_v1_mirror(client):
    response = client.get("/system/hardware-variant")
    mirrored = client.get("/v1/system/hardware-variant")

    assert response.status_code == 200
    assert mirrored.get_json() == response.get_json()
    assert response.get_json() == {
        "variant": "pib5edu",
        "source": "environment",
        "supported": list(ALLOWED_HARDWARE_VARIANTS),
        "implementedVariants": ["pib4edu", "pib5edu"],
        "seedProfileImplemented": True,
    }


def test_hardware_capabilities_shape_and_v1_mirror(client):
    response = client.get("/system/hardware-capabilities")
    mirrored = client.get("/v1/system/hardware-capabilities")

    assert response.status_code == 200
    assert mirrored.get_json() == response.get_json()
    by_kind = {
        capability["kind"]: capability
        for capability in response.get_json()["capabilities"]
    }
    assert set(by_kind) == {
        TINKERFORGE_BRICKLET,
        FEETECH_ST_SERIAL,
        ROBSTRIDE_CAN,
    }
    assert by_kind[TINKERFORGE_BRICKLET]["installedControllers"] == 8
    assert by_kind[FEETECH_ST_SERIAL]["installedControllers"] == 0
    assert by_kind[ROBSTRIDE_CAN]["installedControllers"] == 0
    assert by_kind[TINKERFORGE_BRICKLET]["feedback"] == [
        "current",
        "target_position",
    ]
    assert "current_limit" not in by_kind[TINKERFORGE_BRICKLET]["meaningfulSettings"]
    assert "current_limit" in by_kind[FEETECH_ST_SERIAL]["meaningfulSettings"]
    assert "torque_limit" in by_kind[ROBSTRIDE_CAN]["meaningfulSettings"]


def test_properties_shape_v1_mirror_and_registry_filter(app, client):
    with app.app_context():
        db.session.add(
            SystemProperty(
                key="unregistered.internal",
                value="hidden",
                value_type="str",
                source="default",
            )
        )
        db.session.commit()

    response = client.get("/system/properties")
    mirrored = client.get("/v1/system/properties")

    assert response.status_code == 200
    assert mirrored.get_json() == response.get_json()
    properties = response.get_json()["properties"]
    assert properties
    assert all(
        set(item) == {"key", "value", "valueType", "source", "updatedAt"}
        for item in properties
    )
    assert "unregistered.internal" not in {item["key"] for item in properties}


@pytest.mark.parametrize(
    "path",
    [
        "/system/hardware-variant",
        "/system/hardware-capabilities",
        "/system/properties",
    ],
)
@pytest.mark.parametrize("method", ["PUT", "POST"])
def test_system_fact_endpoints_are_read_only(client, path, method):
    before = client.get("/system/properties").get_json()["properties"]

    response = client.open(path, method=method, json={})

    assert response.status_code == 405
    assert "GET" in response.headers["Allow"]
    assert client.get("/system/properties").get_json()["properties"] == before


def test_long_standing_route_rejects_unsupported_method(client):
    response = client.put("/motor")

    assert response.status_code == 405
    assert "GET" in response.headers["Allow"]


def test_method_not_allowed_is_logged_without_traceback(client, caplog):
    with caplog.at_level(logging.ERROR):
        response = client.put("/system/properties", json={})

    assert response.status_code == 405
    assert "Traceback" not in caplog.text
    assert len(caplog.records) == 1


def test_diagnostics_contains_hardware_variant(client):
    response = client.get("/api/v1/diagnostics/summary")

    assert response.status_code == 200
    assert response.get_json()["hardwareVariant"] == "pib5edu"


def test_hardware_variant_without_stored_row_falls_back(app, client, monkeypatch):
    monkeypatch.setenv("PIB_HARDWARE_VARIANT", "pib5edu")
    with app.app_context():
        db.session.query(SystemProperty).filter_by(key=HARDWARE_VARIANT_KEY).delete()
        db.session.commit()

    response = client.get("/system/hardware-variant")

    assert response.status_code == 200
    assert response.get_json()["variant"] == "pib5edu"
    assert response.get_json()["source"] == "environment"


def test_hardware_variant_never_reports_unknown_stored_value(app, client):
    with app.app_context():
        variant = db.session.get(SystemProperty, HARDWARE_VARIANT_KEY)
        variant.value = "unknown-robot"
        db.session.commit()

    response = client.get("/system/hardware-variant")

    assert response.status_code == 200
    assert response.get_json()["variant"] == "pib5edu"
    assert response.get_json()["source"] == "fallback"


def test_variant_client_calls_flask_test_client(client, monkeypatch):
    from pib_api_client import variant_client

    def send_to_test_client(request):
        response = client.open(
            urlsplit(request.full_url).path,
            method=request.get_method(),
        )
        return response.status_code == 200, response.get_json()

    monkeypatch.setattr(variant_client, "send_request", send_to_test_client)

    success, payload = variant_client.get_hardware_variant()

    assert success is True
    assert payload["variant"] == "pib5edu"
