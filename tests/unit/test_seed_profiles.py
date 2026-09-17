"""Contract tests for educational hardware seed profiles."""

from pathlib import Path

import pytest

import seed_profiles
from seed_profiles import (
    PROFILES,
    UnknownHardwareVariantError,
    get_profile,
    resolve_variant_from_environment,
)

TINKERFORGE = "tinkerforge_bricklet"
SERVO = "Servo Bricklet"
SSR = "Solid State Relay Bricklet"
RGB_BUTTON = "RGB LED Button Bricklet"


@pytest.mark.parametrize(
    ("variant", "expected_controllers"),
    [
        (
            "pib4edu",
            (
                (1, TINKERFORGE, SERVO, 7.5),
                (2, TINKERFORGE, SERVO, 7.5),
                (3, TINKERFORGE, SERVO, 7.5),
                (4, TINKERFORGE, SSR, None),
                (5, TINKERFORGE, RGB_BUTTON, None),
                (6, TINKERFORGE, RGB_BUTTON, None),
                (7, TINKERFORGE, RGB_BUTTON, None),
            ),
        ),
        (
            "pib5edu",
            (
                (1, TINKERFORGE, SERVO, 7.5),
                (2, TINKERFORGE, SERVO, 7.5),
                (3, TINKERFORGE, SERVO, 7.5),
                (4, TINKERFORGE, SERVO, 12.0),
                (5, TINKERFORGE, SSR, None),
                (6, TINKERFORGE, RGB_BUTTON, None),
                (7, TINKERFORGE, RGB_BUTTON, None),
                (8, TINKERFORGE, RGB_BUTTON, None),
            ),
        ),
    ],
)
def test_profile_controller_data(variant: str, expected_controllers: tuple):
    profile = get_profile(variant)

    actual = tuple(
        (
            controller.number,
            controller.kind,
            controller.device_type,
            controller.supply_voltage,
        )
        for controller in profile.controllers
    )
    assert actual == expected_controllers
    assert all(controller.address is None for controller in profile.controllers)


@pytest.mark.parametrize("variant", ["pib4edu", "pib5edu"])
def test_motor_mapping_has_26_unique_motors_on_existing_controllers(variant: str):
    profile = get_profile(variant)
    controller_numbers = {controller.number for controller in profile.controllers}

    assert len(profile.motor_mapping) == 26
    assert len(set(profile.motor_mapping)) == 26
    assert {
        controller_number
        for controller_number, _channel in profile.motor_mapping.values()
    } <= controller_numbers


def test_pib5edu_mapping_differs_by_exactly_four_relocated_motors():
    pib4_mapping = get_profile("pib4edu").motor_mapping
    pib5_mapping = get_profile("pib5edu").motor_mapping

    differences = {
        name: (pib4_mapping[name], pib5_mapping[name])
        for name in pib4_mapping
        if pib4_mapping[name] != pib5_mapping[name]
    }
    assert differences == {
        "shoulder_vertical_right": ((2, 1), (4, 0)),
        "shoulder_vertical_left": ((2, 9), (4, 1)),
        "elbow_left": ((3, 8), (4, 2)),
        "elbow_right": ((1, 8), (4, 3)),
    }


def test_rgb_button_controller_ids_are_variant_specific():
    assert get_profile("pib4edu").rgb_button_controller_ids == (5, 6, 7)
    assert get_profile("pib5edu").rgb_button_controller_ids == (6, 7, 8)


def test_edu_profiles_share_motor_parameters():
    pib4_profile = get_profile("pib4edu")
    pib5_profile = get_profile("pib5edu")

    assert (
        pib4_profile.motor_parameter_defaults is pib5_profile.motor_parameter_defaults
    )
    assert (
        pib4_profile.motor_parameter_deviations
        is pib5_profile.motor_parameter_deviations
    )


@pytest.mark.parametrize("variant", ["pib4advanced", "pib5advanced", "pib5museum"])
def test_unimplemented_variant_error_lists_implemented_profiles(variant: str):
    with pytest.raises(UnknownHardwareVariantError) as error:
        get_profile(variant)

    assert variant in str(error.value)
    assert all(profile_name in str(error.value) for profile_name in PROFILES)


def test_variant_resolution_prefers_nonempty_environment(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
):
    variant_file = tmp_path / "pib_hardware_variant"
    variant_file.write_text("pib4edu\n", encoding="utf-8")
    monkeypatch.setattr(seed_profiles, "HARDWARE_VARIANT_FILE", variant_file)
    monkeypatch.setenv("PIB_HARDWARE_VARIANT", "  pib5edu ")

    assert resolve_variant_from_environment() == "pib5edu"


def test_variant_resolution_uses_file_for_empty_environment(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
):
    variant_file = tmp_path / "pib_hardware_variant"
    variant_file.write_text(" pib4edu\n", encoding="utf-8")
    monkeypatch.setattr(seed_profiles, "HARDWARE_VARIANT_FILE", variant_file)
    monkeypatch.setenv("PIB_HARDWARE_VARIANT", " \t")

    assert resolve_variant_from_environment() == "pib4edu"


def test_variant_resolution_defaults_without_environment_or_file(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
):
    monkeypatch.setattr(
        seed_profiles, "HARDWARE_VARIANT_FILE", tmp_path / "missing-variant-file"
    )
    monkeypatch.delenv("PIB_HARDWARE_VARIANT", raising=False)

    assert resolve_variant_from_environment() == "pib5edu"


def test_profile_literals_match_the_controller_model(app):
    """the profiles keep their own literals to stay importable without the Flask app; guard the drift"""
    from model.controller_model import (
        RGB_LED_BUTTON_BRICKLET,
        SERVO_BRICKLET,
        SOLID_STATE_RELAY_BRICKLET,
        TINKERFORGE_BRICKLET,
    )

    for variant in PROFILES:
        profile = get_profile(variant)
        assert {controller.kind for controller in profile.controllers} == {
            TINKERFORGE_BRICKLET
        }
        assert {controller.device_type for controller in profile.controllers} == {
            SERVO_BRICKLET,
            SOLID_STATE_RELAY_BRICKLET,
            RGB_LED_BUTTON_BRICKLET,
        }
