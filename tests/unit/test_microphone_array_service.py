"""Unit tests for the retired backend microphone-array owner."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

from app.app import db
from model.system_property_model import SystemProperty
from seed_profiles.edu_microphone_tuning import MICROPHONE_TUNING
from service import microphone_array_service as mas
from service.system_property_service import MICROPHONE_DESIRED_STATE_KEY

ROS_AUDIO_PACKAGE = Path(__file__).parents[2] / "ros_packages" / "ros_audio_io"
sys.path.insert(0, str(ROS_AUDIO_PACKAGE))

from ros_audio_io.microphone_parameters import PRESETS as OWNER_PRESETS  # noqa: E402


@pytest.fixture(autouse=True)
def _database_context(app_ctx):
    yield


def test_list_presets_includes_required_names():
    presets = mas.get_service().list_presets()
    for name in (
        "Standard",
        "Noisy Environment / ASR",
        "Loud Speaker Playback",
        "Raw",
        "Custom",
    ):
        assert name in presets


def test_legacy_tuning_is_explicitly_simulated():
    tuning = mas.get_tuning()
    assert tuning["preset"] == "Standard"
    assert tuning["simulation"] is True
    assert tuning["legacy"] is True
    assert tuning["applied_to_device"] is False
    assert tuning["simulation_reason"] == mas.SIMULATION_REASON
    assert tuning["control_surface"] == "ROS 2 parameters via rosbridge"


def test_legacy_updates_only_simulated_cache():
    tuning = mas.update_tuning(
        {
            "preset": "Noisy Environment / ASR",
            "led_ring": {"mode": "listen", "brightness": 24},
        }
    )
    assert tuning["preset"] == "Noisy Environment / ASR"
    assert tuning["parameters"]["AGCONOFF"] == 0
    assert tuning["led_ring"]["mode"] == "listen"
    assert tuning["led_ring"]["brightness"] == 24
    assert tuning["applied_to_device"] is False


def test_profile_default_matches_device_owner_standard_preset():
    assert dict(MICROPHONE_TUNING) == OWNER_PRESETS["Standard"]


def test_missing_desired_state_is_created_from_profile_default():
    db.session.query(SystemProperty).filter_by(
        key=MICROPHONE_DESIRED_STATE_KEY
    ).delete()
    db.session.commit()

    desired_state = mas.MicrophoneArrayService().get_desired_state()

    assert desired_state["parameters"] == MICROPHONE_TUNING
    assert desired_state["preset"] == "Standard"
    assert desired_state["revision"] == 1
    assert desired_state["updatedAt"]


def test_tuning_and_led_survive_a_fresh_service_instance():
    first_service = mas.MicrophoneArrayService()
    first_service.update_tuning(
        {
            "parameters": {"HPFONOFF": 3},
            "led_ring": {"mode": "spin", "brightness": 20, "color": "#12abef"},
        }
    )
    db.session.commit()

    desired_state = mas.MicrophoneArrayService().get_desired_state()

    assert desired_state["parameters"]["HPFONOFF"] == 3
    assert desired_state["led_ring"] == {
        "mode": "spin",
        "brightness": 20,
        "color": "#12ABEF",
        "vad_led": 0,
    }
    assert desired_state["preset"] == "Custom"
    assert desired_state["revision"] == 2


def test_apply_raw_preset_and_alias():
    assert mas.update_tuning({"preset": "Raw"})["parameters"]["ECHOONOFF"] == 0
    assert mas.update_tuning({"preset": "Raw Pass-Through"})["preset"] == "Raw"


def test_custom_parameter_update_sets_custom_preset():
    tuning = mas.update_tuning({"parameters": {"AGCONOFF": 0, "HPFONOFF": 3}})
    assert tuning["preset"] == "Custom"
    assert tuning["parameters"]["AGCONOFF"] == 0
    assert tuning["parameters"]["HPFONOFF"] == 3


def test_invalid_parameter_raises():
    with pytest.raises(ValueError, match="Unknown parameter"):
        mas.update_tuning({"parameters": {"NOT_A_PARAM": 1}})


def test_out_of_range_parameter_raises():
    with pytest.raises(ValueError, match="out of range"):
        mas.update_tuning({"parameters": {"HPFONOFF": 9}})


def test_read_only_parameter_rejected():
    with pytest.raises(ValueError, match="read-only"):
        mas.update_tuning({"parameters": {"DOAANGLE": 90}})


def test_unknown_preset_raises():
    with pytest.raises(ValueError, match="Unknown preset"):
        mas.update_tuning({"preset": "Does Not Exist"})


def test_invalid_led_values_raise():
    with pytest.raises(ValueError, match="Unknown LED mode"):
        mas.update_tuning({"led_ring": {"mode": "disco"}})
    with pytest.raises(ValueError, match="brightness"):
        mas.update_tuning({"led_ring": {"brightness": 32}})
    with pytest.raises(ValueError, match="RRGGBB"):
        mas.update_tuning({"led_ring": {"color": "#xyz"}})


def test_telemetry_contains_no_fabricated_measurements():
    telemetry = mas.get_telemetry()
    assert telemetry["doa_angle"] is None
    assert telemetry["voice_activity"] is None
    assert telemetry["speech_detected"] is None
    assert telemetry["audio_levels"] == []
    assert telemetry["simulation"] is True
    assert telemetry["simulation_reason"] == mas.SIMULATION_REASON


def test_health_reports_ros_owner_and_led_control():
    health = mas.health()
    assert health["simulation"] is True
    assert health["simulation_reason"] == mas.SIMULATION_REASON
    assert health["device_access"] is False
    assert health["owner"] == "ros-audio-io"
    assert health["led_owner"] == "ros-audio-io"
    assert health["led_control"] == "ROS 2 parameters via rosbridge"
