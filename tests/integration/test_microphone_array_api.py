"""Integration tests for microphone-array REST API (PR-1519)."""

from __future__ import annotations

import pytest
from click.testing import CliRunner

from app.app import app, db  # noqa: E402
from commands import seed_db  # noqa: E402
from seed_profiles import get_profile  # noqa: E402
from service import microphone_array_service as mas  # noqa: E402


@pytest.fixture
def client(app):
    app.config["TESTING"] = True
    with app.test_client() as test_client:
        yield test_client


def test_get_health(client):
    response = client.get("/system/microphone-array/health")
    assert response.status_code == 200
    data = response.get_json()
    assert data["simulation"] is True
    assert data["simulation_reason"] == mas.SIMULATION_REASON
    assert data["device_access"] is False
    assert data["owner"] == "ros-audio-io"
    assert data["led_owner"] == "ros-audio-io"
    assert data["led_control"] == "ROS 2 parameters via rosbridge"


def test_get_health_v1_prefix(client):
    response = client.get("/v1/system/microphone-array/health")
    assert response.status_code == 200
    assert response.get_json()["owner"] == "ros-audio-io"


def test_get_telemetry(client):
    response = client.get("/system/microphone-array/telemetry")
    assert response.status_code == 200
    data = response.get_json()
    assert data["doa_angle"] is None
    assert data["voice_activity"] is None
    assert data["speech_detected"] is None
    assert data["audio_levels"] == []
    assert data["legacy"] is True
    assert data["simulation_reason"] == mas.SIMULATION_REASON


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
    assert data["legacy"] is True
    assert data["applied_to_device"] is False
    assert data["simulation_reason"] == mas.SIMULATION_REASON


def test_get_desired_state_for_device_owner(client):
    response = client.get("/v1/system/microphone-array/desired-state")

    assert response.status_code == 200
    data = response.get_json()
    assert data["preset"] == "Standard"
    assert "AGCONOFF" in data["parameters"]
    assert data["led_ring"]["mode"] == "off"
    assert data["revision"] == 1
    assert data["updatedAt"]


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

    desired = client.get("/system/microphone-array/desired-state").get_json()
    assert desired["led_ring"]["mode"] == "spin"
    assert desired["led_ring"]["brightness"] == 20
    assert desired["revision"] == 2


def test_changed_state_survives_seed_path_and_fresh_service(client):
    response = client.post(
        "/system/microphone-array/tuning",
        json={"parameters": {"ECHOONOFF": 0}},
    )
    changed = response.get_json()

    with app.app_context():
        result = CliRunner().invoke(seed_db, [])
        assert result.exit_code == 0, result.output
        assert "already contains data" in result.output
        mas.seed_desired_state(get_profile("pib5edu"))
        db.session.expire_all()

        persisted = mas.MicrophoneArrayService().get_desired_state()

    assert persisted["parameters"]["ECHOONOFF"] == 0
    assert persisted["revision"] == 2
    assert changed["parameters"]["ECHOONOFF"] == 0


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
