"""Unit tests for microphone array service (PR-1519)."""

from __future__ import annotations

import os
from unittest.mock import MagicMock, patch

import pytest

# Force simulation before the service module constructs its singleton.
os.environ["MICROPHONE_ARRAY_SIMULATION"] = "1"

from service import microphone_array_service as mas  # noqa: E402


@pytest.fixture(autouse=True)
def _reset_service():
    service = mas.get_service()
    service.reset_for_tests()
    yield
    service.reset_for_tests()


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


def test_default_tuning_is_standard_preset():
    tuning = mas.get_tuning()
    assert tuning["preset"] == "Standard"
    assert tuning["simulation"] is True
    assert tuning["parameters"]["AGCONOFF"] == 1
    assert tuning["parameters"]["STATNOISEONOFF"] == 1
    assert tuning["parameters"]["ECHOONOFF"] == 1
    assert tuning["led_ring"]["mode"] == "off"


def test_apply_noisy_asr_preset():
    tuning = mas.update_tuning({"preset": "Noisy Environment / ASR"})
    assert tuning["preset"] == "Noisy Environment / ASR"
    assert tuning["parameters"]["AGCONOFF"] == 0
    assert tuning["parameters"]["HPFONOFF"] == 2
    assert tuning["parameters"]["STATNOISEONOFF_SR"] == 1


def test_apply_raw_preset_and_alias():
    tuning = mas.update_tuning({"preset": "Raw"})
    assert tuning["preset"] == "Raw"
    assert tuning["parameters"]["AGCONOFF"] == 0
    assert tuning["parameters"]["STATNOISEONOFF"] == 0
    assert tuning["parameters"]["ECHOONOFF"] == 0
    assert tuning["parameters"]["HPFONOFF"] == 0

    aliased = mas.update_tuning({"preset": "Raw Pass-Through"})
    assert aliased["preset"] == "Raw"


def test_loud_speaker_playback_preset_enables_echo():
    tuning = mas.update_tuning({"preset": "Loud Speaker Playback"})
    assert tuning["preset"] == "Loud Speaker Playback"
    assert tuning["parameters"]["ECHOONOFF"] == 1
    assert tuning["parameters"]["AGCONOFF"] == 1


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


def test_led_ring_update_persists_in_simulation():
    tuning = mas.update_tuning(
        {
            "led_ring": {
                "mode": "listen",
                "brightness": 24,
                "color": "#112233",
                "vad_led": True,
            }
        }
    )
    led = tuning["led_ring"]
    assert led["mode"] == "listen"
    assert led["brightness"] == 24
    assert led["color"] == "#112233"
    assert led["vad_led"] == 1


def test_invalid_led_mode_raises():
    with pytest.raises(ValueError, match="Unknown LED mode"):
        mas.update_tuning({"led_ring": {"mode": "disco"}})


def test_telemetry_simulation_shape():
    telemetry = mas.get_telemetry()
    assert telemetry["doa_angle"] == 180
    assert telemetry["voice_activity"] is False
    assert telemetry["speech_detected"] is False
    assert len(telemetry["audio_levels"]) == 5
    assert telemetry["simulation"] is True


def test_respeaker_tuning_write_and_read_int():
    mock_dev = MagicMock()
    mock_dev.ctrl_transfer.return_value = MagicMock(
        tobytes=MagicMock(return_value=struct_pack_ii(1, 0))
    )
    mock_usb = MagicMock()
    mock_usb.util.CTRL_OUT = 0x40
    mock_usb.util.CTRL_IN = 0xC0
    mock_usb.util.CTRL_TYPE_VENDOR = 0x40
    mock_usb.util.CTRL_RECIPIENT_DEVICE = 0x00
    with patch.object(mas, "usb", mock_usb):
        driver = mas.ReSpeakerTuning(mock_dev)
        driver.write("AGCONOFF", 1)
        assert mock_dev.ctrl_transfer.called
        value = driver.read("AGCONOFF")
        assert value == 1


def test_respeaker_tuning_rejects_read_only_write():
    driver = mas.ReSpeakerTuning(MagicMock())
    with pytest.raises(ValueError, match="read-only"):
        driver.write("DOAANGLE", 10)


def test_hardware_path_uses_usb_when_device_present():
    mock_dev = MagicMock()
    # DOAANGLE / VOICEACTIVITY / SPEECHDETECTED style int responses.
    mock_dev.ctrl_transfer.return_value = MagicMock(
        tobytes=MagicMock(return_value=struct_pack_ii(90, 0))
    )

    with patch.dict(os.environ, {"MICROPHONE_ARRAY_SIMULATION": "0"}):
        with patch.object(mas, "_USB_AVAILABLE", True), patch.object(
            mas, "usb"
        ) as mock_usb:
            mock_usb.core.find.return_value = mock_dev
            mock_usb.util.CTRL_OUT = 0x40
            mock_usb.util.CTRL_IN = 0xC0
            mock_usb.util.CTRL_TYPE_VENDOR = 0x40
            mock_usb.util.CTRL_RECIPIENT_DEVICE = 0x00
            service = mas.MicrophoneArrayService()
            assert service.is_simulation is False
            telemetry = service.get_telemetry()
            assert telemetry["doa_angle"] == 90
            assert telemetry["simulation"] is False


def struct_pack_ii(a: int, b: int) -> bytes:
    import struct

    return struct.pack(b"ii", a, b)
