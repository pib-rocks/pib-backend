"""Integration tests for microphone-array REST API (PR-1519)."""

from __future__ import annotations

import os

import pytest

os.environ.setdefault("MICROPHONE_ARRAY_SIMULATION", "1")

from app.app import app  # noqa: E402
from service import microphone_array_service as mas  # noqa: E402


@pytest.fixture
def client():
    mas.get_service().reset_for_tests()
    app.config["TESTING"] = True
    with app.test_client() as test_client:
        yield test_client
    mas.get_service().reset_for_tests()


def test_get_telemetry(client):
    response = client.get("/system/microphone-array/telemetry")
    assert response.status_code == 200
    data = response.get_json()
    assert data["doa_angle"] == 180
    assert data["voice_activity"] is False
    assert data["speech_detected"] is False
    assert isinstance(data["audio_levels"], list)
    assert len(data["audio_levels"]) == 5


def test_get_telemetry_v1_prefix(client):
    response = client.get("/v1/system/microphone-array/telemetry")
    assert response.status_code == 200
    data = response.get_json()
    assert "doa_angle" in data


def test_get_tuning(client):
    response = client.get("/system/microphone-array/tuning")
    assert response.status_code == 200
    data = response.get_json()
    assert data["preset"] == "Standard"
    assert "AGCONOFF" in data["parameters"]
    assert "led_ring" in data
    assert "Standard" in data["presets"]
    assert "Raw" in data["presets"]


def test_post_tuning_preset(client):
    response = client.post(
        "/system/microphone-array/tuning",
        json={"preset": "Noisy Environment / ASR"},
    )
    assert response.status_code == 200
    data = response.get_json()
    assert data["preset"] == "Noisy Environment / ASR"
    assert data["parameters"]["AGCONOFF"] == 0

    # Persistence across GET
    follow = client.get("/system/microphone-array/tuning")
    assert follow.get_json()["preset"] == "Noisy Environment / ASR"


def test_post_tuning_parameters(client):
    response = client.post(
        "/system/microphone-array/tuning",
        json={"parameters": {"ECHOONOFF": 0, "HPFONOFF": 3}},
    )
    assert response.status_code == 200
    data = response.get_json()
    assert data["preset"] == "Custom"
    assert data["parameters"]["ECHOONOFF"] == 0
    assert data["parameters"]["HPFONOFF"] == 3


def test_post_tuning_led_ring(client):
    response = client.post(
        "/system/microphone-array/tuning",
        json={"led_ring": {"mode": "spin", "brightness": 20}},
    )
    assert response.status_code == 200
    led = response.get_json()["led_ring"]
    assert led["mode"] == "spin"
    assert led["brightness"] == 20


def test_post_tuning_invalid_json(client):
    response = client.post(
        "/system/microphone-array/tuning",
        data="not-json",
        content_type="application/json",
    )
    assert response.status_code == 400
    assert "error" in response.get_json()


def test_post_tuning_unknown_preset(client):
    response = client.post(
        "/system/microphone-array/tuning",
        json={"preset": "Galaxy Mode"},
    )
    assert response.status_code == 400
    assert "error" in response.get_json()


def test_post_tuning_out_of_range(client):
    response = client.post(
        "/system/microphone-array/tuning",
        json={"parameters": {"HPFONOFF": 99}},
    )
    assert response.status_code == 400
