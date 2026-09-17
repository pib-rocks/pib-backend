"""Regression tests for GET/PUT /camera-settings (PR-1742)."""

from __future__ import annotations

from schema.camera_settings_schema import camera_settings_schema
from service import camera_service

CAMERA_PAYLOAD = {
    "qualityFactor": 80,
    "refreshRate": 0.1,
    "resX": 640,
    "resY": 480,
    "resolution": "SD",
}


def test_schema_load_uses_snake_case_dto_keys(app):
    with app.app_context():
        dto = camera_settings_schema.load(CAMERA_PAYLOAD)
    assert dto["resolution"] == "SD"
    assert dto["refresh_rate"] == 0.1
    assert dto["quality_factor"] == 80
    assert dto["res_x"] == 640
    assert dto["res_y"] == 480
    assert "refreshRate" not in dto


def test_schema_load_ignores_ui_is_active(app):
    with app.app_context():
        dto = camera_settings_schema.load({**CAMERA_PAYLOAD, "isActive": True})
    assert "is_active" not in dto
    assert "isActive" not in dto


def test_update_camera_settings_assigns_model_columns(app):
    with app.app_context():
        dto = camera_settings_schema.load(
            {
                "qualityFactor": 50,
                "refreshRate": 0.5,
                "resX": 1280,
                "resY": 720,
                "resolution": "HD",
            }
        )
        updated = camera_service.update_camera_settings(dto)
        assert updated.resolution == "HD"
        assert updated.refresh_rate == 0.5
        assert updated.quality_factor == 50
        assert updated.res_x == 1280
        assert updated.res_y == 720


def test_put_camera_settings_persists_and_get_returns_new_values(app):
    payload = {
        "qualityFactor": 50,
        "refreshRate": 0.5,
        "resX": 1280,
        "resY": 720,
        "resolution": "HD",
    }
    with app.test_client() as client:
        put_response = client.put("/camera-settings", json=payload)
        assert put_response.status_code == 200
        put_body = put_response.get_json()
        assert put_body["qualityFactor"] == 50
        assert put_body["refreshRate"] == 0.5
        assert put_body["resX"] == 1280
        assert put_body["resY"] == 720
        assert put_body["resolution"] == "HD"

        get_response = client.get("/camera-settings")
        assert get_response.status_code == 200
        assert get_response.get_json() == put_body


def test_put_camera_settings_ignores_is_active(app):
    with app.test_client() as client:
        response = client.put(
            "/camera-settings",
            json={**CAMERA_PAYLOAD, "isActive": False},
        )
        assert response.status_code == 200
        body = response.get_json()
        assert "isActive" not in body
        assert body["qualityFactor"] == 80
        assert body["resolution"] == "SD"
