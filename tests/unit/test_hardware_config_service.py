"""Unit tests for hardware config export/import (PR-1527)."""

from __future__ import annotations

import pytest

from model.bricklet_model import Bricklet
from model.motor_model import Motor
from service import hardware_config_service as hcs


@pytest.fixture()
def seeded(app_ctx):
    """Ensure seed data is present; yield nothing."""
    assert Bricklet.query.count() >= 7
    assert Motor.query.count() >= 1
    yield


def _set_uid(bricklet_number: int, uid: str | None) -> None:
    bricklet = Bricklet.query.filter(Bricklet.bricklet_number == bricklet_number).one()
    bricklet.uid = uid
    from app.app import db

    db.session.flush()


def test_export_includes_bricklets_and_motors(seeded):
    _set_uid(1, "SRV001")
    _set_uid(4, "REL001")

    document = hcs.export_hardware_config()

    assert document["version"] == 1
    assert isinstance(document["bricklets"], list)
    assert isinstance(document["motors"], list)
    assert len(document["bricklets"]) >= 7
    assert len(document["motors"]) >= 1

    by_number = {b["brickletNumber"]: b for b in document["bricklets"]}
    assert by_number[1]["uid"] == "SRV001"
    assert by_number[1]["type"] == "Servo Bricklet"
    assert by_number[4]["uid"] == "REL001"
    assert by_number[4]["type"] == "Solid State Relay Bricklet"

    elbow = next(m for m in document["motors"] if m["name"] == "elbow_left")
    assert "pulseWidthMin" in elbow
    assert "rotationRangeMax" in elbow
    assert isinstance(elbow["brickletPins"], list)
    assert elbow["brickletPins"][0]["brickletNumber"] == 3
    assert elbow["brickletPins"][0]["pin"] == 8


def test_import_updates_uids_and_motor_limits(seeded):
    document = hcs.export_hardware_config()
    for bricklet in document["bricklets"]:
        if bricklet["brickletNumber"] == 1:
            bricklet["uid"] = "NEW001"
        if bricklet["brickletNumber"] == 2:
            bricklet["uid"] = "NEW002"
    for motor in document["motors"]:
        if motor["name"] == "elbow_left":
            motor["velocity"] = 12345
            motor["rotationRangeMin"] = -1000
            motor["rotationRangeMax"] = 1000

    result = hcs.import_hardware_config(document)

    assert Bricklet.query.filter_by(bricklet_number=1).one().uid == "NEW001"
    assert Bricklet.query.filter_by(bricklet_number=2).one().uid == "NEW002"
    elbow = Motor.query.filter_by(name="elbow_left").one()
    assert elbow.velocity == 12345
    assert elbow.rotation_range_min == -1000
    assert elbow.rotation_range_max == 1000

    by_number = {b["brickletNumber"]: b for b in result["bricklets"]}
    assert by_number[1]["uid"] == "NEW001"


def test_import_swaps_uids_without_unique_conflict(seeded):
    _set_uid(1, "AAA111")
    _set_uid(2, "BBB222")
    document = hcs.export_hardware_config()
    for bricklet in document["bricklets"]:
        if bricklet["brickletNumber"] == 1:
            bricklet["uid"] = "BBB222"
        if bricklet["brickletNumber"] == 2:
            bricklet["uid"] = "AAA111"

    hcs.import_hardware_config(document)

    assert Bricklet.query.filter_by(bricklet_number=1).one().uid == "BBB222"
    assert Bricklet.query.filter_by(bricklet_number=2).one().uid == "AAA111"


def test_import_rejects_invalid_uid(seeded):
    document = hcs.export_hardware_config()
    document["bricklets"][0]["uid"] = "BAD_UID!"

    with pytest.raises(ValueError, match="invalid format"):
        hcs.import_hardware_config(document)


def test_import_rejects_uid_too_long(seeded):
    document = hcs.export_hardware_config()
    document["bricklets"][0]["uid"] = "TOOLONG"

    with pytest.raises(ValueError, match="invalid format"):
        hcs.import_hardware_config(document)


def test_import_rejects_duplicate_uids(seeded):
    document = hcs.export_hardware_config()
    document["bricklets"][0]["uid"] = "SAME01"
    document["bricklets"][1]["uid"] = "SAME01"

    with pytest.raises(ValueError, match="Duplicate Bricklet UID"):
        hcs.import_hardware_config(document)


def test_import_rejects_missing_bricklets_array(seeded):
    with pytest.raises(ValueError, match="bricklets"):
        hcs.import_hardware_config({"version": 1, "motors": []})


def test_import_rejects_unsupported_version(seeded):
    with pytest.raises(ValueError, match="Unsupported hardware config version"):
        hcs.import_hardware_config(
            {"version": 99, "bricklets": [], "motors": []}
        )


def test_import_rejects_unknown_bricklet_type(seeded):
    document = hcs.export_hardware_config()
    document["bricklets"][0]["type"] = "Flux Capacitor Bricklet"

    with pytest.raises(ValueError, match="not a supported Bricklet type"):
        hcs.import_hardware_config(document)


def test_import_rejects_unknown_motor(seeded):
    document = hcs.export_hardware_config()
    document["motors"].append(
        {
            "name": "does_not_exist",
            "velocity": 1,
            "brickletPins": [],
        }
    )

    with pytest.raises(ValueError, match="Unknown motor"):
        hcs.import_hardware_config(document)


def test_roundtrip_preserves_seeded_types(seeded):
    original = hcs.export_hardware_config()
    types_before = {b["brickletNumber"]: b["type"] for b in original["bricklets"]}

    restored = hcs.import_hardware_config(original)
    types_after = {b["brickletNumber"]: b["type"] for b in restored["bricklets"]}

    assert types_before == types_after
    assert set(types_before.values()) >= {
        "Servo Bricklet",
        "Solid State Relay Bricklet",
        "RGB LED Button Bricklet",
    }


def test_import_clears_uid_when_empty_string(seeded):
    _set_uid(1, "CLR001")
    document = hcs.export_hardware_config()
    for bricklet in document["bricklets"]:
        if bricklet["brickletNumber"] == 1:
            bricklet["uid"] = ""

    hcs.import_hardware_config(document)
    assert Bricklet.query.filter_by(bricklet_number=1).one().uid is None


def test_validate_accepts_snake_case_aliases(seeded):
    document = {
        "version": 1,
        "bricklets": [
            {"bricklet_number": 1, "uid": "SNK001", "type": "Servo Bricklet"}
        ],
        "motors": [
            {
                "name": "elbow_left",
                "pulse_width_min": 800,
                "bricklet_pins": [
                    {"bricklet_number": 3, "pin": 8, "invert": False}
                ],
            }
        ],
    }
    validated = hcs.validate_hardware_config(document)
    assert validated["bricklets"][0]["bricklet_number"] == 1
    assert validated["motors"][0]["settings"]["pulse_width_min"] == 800
