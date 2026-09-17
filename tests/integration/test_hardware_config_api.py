"""Integration tests for hardware-config REST API (PR-1527)."""

from __future__ import annotations

import json

import pytest

from model.controller_model import Controller
from model.motor_model import Motor
from service import hardware_config_service as hcs


@pytest.fixture()
def client(app):
    with app.test_client() as test_client:
        with app.app_context():
            yield test_client


def test_export_endpoint_returns_attachment(client):
    from app.app import db

    with client.application.app_context():
        controller = Controller.query.filter_by(number=1).one()
        controller.address = "EXP001"
        db.session.commit()

    response = client.get("/api/system/hardware-config/export")
    assert response.status_code == 200
    assert response.mimetype == "application/json"
    assert "hardware-config.json" in response.headers.get("Content-Disposition", "")

    data = response.get_json()
    assert data["version"] == 2
    assert any(
        c["number"] == 1 and c["address"] == "EXP001" for c in data["controllers"]
    )
    assert any(m["name"] == "elbow_left" for m in data["motors"])


def test_export_available_on_system_prefix(client):
    response = client.get("/system/hardware-config/export")
    assert response.status_code == 200
    assert "controllers" in response.get_json()


def test_import_endpoint_updates_database(client):
    with client.application.app_context():
        document = hcs.export_hardware_config()
    for controller in document["controllers"]:
        if controller["number"] == 1:
            controller["address"] = "IMP999"
    for motor in document["motors"]:
        if motor["name"] == "tilt_forward_motor":
            motor["velocity"] = 4242

    response = client.post(
        "/api/system/hardware-config/import",
        data=json.dumps(document),
        content_type="application/json",
    )
    assert response.status_code == 200
    body = response.get_json()
    assert any(c["address"] == "IMP999" for c in body["controllers"])

    with client.application.app_context():
        assert Controller.query.filter_by(number=1).one().address == "IMP999"
        assert Motor.query.filter_by(name="tilt_forward_motor").one().velocity == 4242


def test_import_endpoint_rejects_invalid_schema(client):
    response = client.post(
        "/api/system/hardware-config/import",
        json={
            "version": 1,
            "bricklets": [
                {"brickletNumber": 1, "uid": "!!bad!!", "type": "Servo Bricklet"}
            ],
            "motors": [],
        },
    )
    assert response.status_code == 400
    assert "error" in response.get_json()


def test_import_endpoint_rejects_invalid_json_body(client):
    response = client.post(
        "/api/system/hardware-config/import",
        data="not-json",
        content_type="application/json",
    )
    assert response.status_code == 400
    assert response.get_json()["error"] == "Request body must be valid JSON"


def test_import_endpoint_rejects_duplicate_uids(client):
    response = client.post(
        "/api/system/hardware-config/import",
        json={
            "version": 1,
            "bricklets": [
                {"brickletNumber": 1, "uid": "DUP001", "type": "Servo Bricklet"},
                {"brickletNumber": 2, "uid": "DUP001", "type": "Servo Bricklet"},
            ],
            "motors": [],
        },
    )
    assert response.status_code == 400
    assert "Duplicate" in response.get_json()["error"]
