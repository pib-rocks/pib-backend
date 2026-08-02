"""Integration tests for hardware-config REST API (PR-1527)."""

from __future__ import annotations

import json

import pytest

from model.bricklet_model import Bricklet
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
        bricklet = Bricklet.query.filter_by(bricklet_number=1).one()
        bricklet.uid = "EXP001"
        db.session.commit()

    response = client.get("/api/system/hardware-config/export")
    assert response.status_code == 200
    assert response.mimetype == "application/json"
    assert "hardware-config.json" in response.headers.get("Content-Disposition", "")

    data = response.get_json()
    assert data["version"] == 1
    assert any(b["brickletNumber"] == 1 and b["uid"] == "EXP001" for b in data["bricklets"])
    assert any(m["name"] == "elbow_left" for m in data["motors"])


def test_export_available_on_system_prefix(client):
    response = client.get("/system/hardware-config/export")
    assert response.status_code == 200
    assert "bricklets" in response.get_json()


def test_import_endpoint_updates_database(client):
    with client.application.app_context():
        document = hcs.export_hardware_config()
    for bricklet in document["bricklets"]:
        if bricklet["brickletNumber"] == 1:
            bricklet["uid"] = "IMP999"
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
    assert any(b["uid"] == "IMP999" for b in body["bricklets"])

    with client.application.app_context():
        assert Bricklet.query.filter_by(bricklet_number=1).one().uid == "IMP999"
        assert Motor.query.filter_by(name="tilt_forward_motor").one().velocity == 4242


def test_import_endpoint_rejects_invalid_schema(client):
    response = client.post(
        "/api/system/hardware-config/import",
        json={
            "version": 1,
            "bricklets": [{"brickletNumber": 1, "uid": "!!bad!!", "type": "Servo Bricklet"}],
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
