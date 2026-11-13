"""Partial motor-settings requests keep the values the caller did not send (PR-2319).

Measured on 192.168.1.172 before the fix: ``Write.set('pinky_left_stretch',
velocity=100000)`` applied

    {'visible': False, 'upperEndpoint': '', 'rotationRangeMin': 0, 'rotationRangeMax': 0,
     'pulseWidthMin': 0, 'pulseWidthMax': 0, 'velocity': 100000, 'acceleration': 0,
     'deceleration': 0, 'period': 19500, 'turnedOn': False}

i.e. it zeroed the pulse widths and rotation ranges and disabled the motor, while
returning success. ``ApplyMotorSettings.apply_fields`` names the fields a caller wants
to change; this module merges them onto the motor's current settings.
"""

import importlib.util
from pathlib import Path

MODULE_PATH = (
    Path(__file__).resolve().parents[2]
    / "ros_packages"
    / "motors"
    / "motors"
    / "motor_settings_merge.py"
)


def _load_merge_module():
    spec = importlib.util.spec_from_file_location("motor_settings_merge", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


merge_module = _load_merge_module()

CURRENT = {
    "name": "pinky_left_stretch",
    "visible": True,
    "upperEndpoint": "min",
    "rotationRangeMin": -9000,
    "rotationRangeMax": 9000,
    "pulseWidthMin": 750,
    "pulseWidthMax": 2500,
    "velocity": 100000,
    "acceleration": 50000,
    "deceleration": 50000,
    "period": 19500,
    "turnedOn": True,
}

# What the node builds from a request that only carried velocity: every other field is
# the message default.
INCOMING = {
    "name": "pinky_left_stretch",
    "visible": False,
    "upperEndpoint": "",
    "rotationRangeMin": 0,
    "rotationRangeMax": 0,
    "pulseWidthMin": 0,
    "pulseWidthMax": 0,
    "velocity": 6000,
    "acceleration": 0,
    "deceleration": 0,
    "period": 0,
    "turnedOn": False,
}


def test_partial_request_changes_only_the_named_field():
    merged = merge_module.merge_partial_settings(CURRENT, INCOMING, ["velocity"])

    assert merged["velocity"] == 6000
    for key in (
        "acceleration",
        "deceleration",
        "pulseWidthMin",
        "pulseWidthMax",
        "rotationRangeMin",
        "rotationRangeMax",
        "period",
        "turnedOn",
        "visible",
        "upperEndpoint",
    ):
        assert merged[key] == CURRENT[key], key
    assert merged["name"] == "pinky_left_stretch"


def test_two_named_fields_are_applied_together():
    merged = merge_module.merge_partial_settings(
        CURRENT, {**INCOMING, "acceleration": 8000}, ["velocity", "acceleration"]
    )

    assert merged["velocity"] == 6000
    assert merged["acceleration"] == 8000
    assert merged["deceleration"] == CURRENT["deceleration"]
    assert merged["turnedOn"] is True


def test_an_explicit_zero_is_applied_when_the_field_is_named():
    merged = merge_module.merge_partial_settings(
        CURRENT, {**INCOMING, "deceleration": 0}, ["deceleration"]
    )

    assert merged["deceleration"] == 0
    assert merged["velocity"] == CURRENT["velocity"]


def test_empty_apply_fields_keeps_the_historical_full_document_behaviour():
    merged = merge_module.merge_partial_settings(CURRENT, INCOMING, [])

    assert merged == INCOMING


def test_unknown_field_names_are_refused_rather_than_ignored():
    try:
        merge_module.merge_partial_settings(CURRENT, INCOMING, ["velocty"])
    except merge_module.UnknownApplyFields as error:
        assert "velocty" in str(error)
    else:  # pragma: no cover - the merge must not silently accept a typo
        raise AssertionError("a misspelled apply_fields entry must raise")


def test_unknown_apply_fields_is_empty_for_the_known_names():
    assert merge_module.unknown_apply_fields([]) == []
    assert merge_module.unknown_apply_fields(list(merge_module.APPLYABLE_FIELDS)) == []
    assert merge_module.unknown_apply_fields(["motor_name"]) == ["motor_name"]


def test_every_motor_settings_field_is_writable():
    """The mapping must cover the message, otherwise a field can never be set alone."""
    message_fields = set(_message_field_names())
    assert message_fields - {"motor_name"} == set(merge_module.APPLYABLE_FIELDS)


def _message_field_names():
    srv_path = (
        Path(__file__).resolve().parents[2]
        / "ros_packages"
        / "datatypes"
        / "msg"
        / "MotorSettings.msg"
    )
    names = []
    for line in srv_path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        names.append(line.split()[-1])
    return names
