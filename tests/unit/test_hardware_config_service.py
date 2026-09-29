"""Unit tests for hardware-config schema version 2 and version 1 imports."""

import pytest

from app.app import db
from model.controller_model import Controller
from model.motor_model import Motor
from service import hardware_config_service as hcs


@pytest.fixture()
def seeded(app_ctx):
    assert Controller.query.count() == 8
    assert Motor.query.count() >= 1


def test_v2_roundtrip_preserves_controllers_mappings_and_limits(seeded):
    controller = Controller.query.filter_by(number=3).one()
    controller.address = "SRV003"
    controller.device_type = "RGB LED Button Bricklet"
    controller.supply_voltage = 12.0
    elbow = Motor.query.filter_by(name="elbow_left").one()
    elbow.current_limit = 1.25
    elbow.torque_limit = 2.5
    db.session.flush()

    exported = hcs.export_hardware_config("pib5edu")
    restored = hcs.import_hardware_config(exported)

    assert restored["version"] == 2
    assert restored["variant"] == "pib5edu"
    assert "bricklets" not in restored
    by_number = {item["number"]: item for item in restored["controllers"]}
    assert by_number[3] == {
        "kind": "tinkerforge_bricklet",
        "deviceType": "RGB LED Button Bricklet",
        "address": "SRV003",
        "number": 3,
        "supplyVoltage": 12.0,
    }
    elbow_dto = next(
        item for item in restored["motors"] if item["name"] == "elbow_left"
    )
    assert elbow_dto["controllerNumber"] == 4
    assert elbow_dto["channel"] == 2
    assert elbow_dto["currentLimit"] == 1.25
    assert elbow_dto["torqueLimit"] == 2.5


def test_v1_import_updates_tinkerforge_addresses_and_motor_mapping(seeded):
    document = {
        "version": 1,
        "bricklets": [
            {
                "brickletNumber": 1,
                "uid": "REL111",
                "type": "Solid State Relay Bricklet",
            },
            {"brickletNumber": 3, "uid": "SRV333", "type": "Servo Bricklet"},
        ],
        "motors": [
            {
                "name": "elbow_left",
                "velocity": 12345,
                "invert": True,
                "brickletPins": [{"brickletNumber": 1, "pin": 6, "invert": False}],
            }
        ],
    }

    result = hcs.import_hardware_config(document)

    assert result["version"] == 2
    assert Controller.query.filter_by(number=1).one().address == "REL111"
    assert (
        Controller.query.filter_by(number=1).one().device_type
        == "Solid State Relay Bricklet"
    )
    elbow = Motor.query.filter_by(name="elbow_left").one()
    assert elbow.controller.number == 1
    assert elbow.channel == 6
    assert elbow.velocity == 12345
    assert elbow.invert is True


def test_import_swaps_controller_addresses_without_unique_conflict(seeded):
    first = Controller.query.filter_by(number=1).one()
    second = Controller.query.filter_by(number=2).one()
    first.address = "AAA111"
    second.address = "BBB222"
    db.session.flush()
    document = hcs.export_hardware_config()
    document["controllers"][0]["address"] = "BBB222"
    document["controllers"][1]["address"] = "AAA111"

    hcs.import_hardware_config(document)

    assert first.address == "BBB222"
    assert second.address == "AAA111"


def test_import_accepts_all_registered_controller_kinds(seeded):
    document = hcs.export_hardware_config()
    document["controllers"][0].update(
        {
            "kind": "feetech_st_serial",
            "deviceType": None,
            "address": "/dev/ttyUSB0",
            "supplyVoltage": 12,
        }
    )
    document["controllers"][1].update(
        {
            "kind": "robstride_can",
            "deviceType": None,
            "address": "can0",
            "supplyVoltage": 48.0,
        }
    )

    hcs.import_hardware_config(document)

    assert Controller.query.filter_by(number=1).one().kind == "feetech_st_serial"
    assert Controller.query.filter_by(number=2).one().kind == "robstride_can"


def test_import_rejects_unknown_controller_kind(seeded):
    document = hcs.export_hardware_config()
    document["controllers"][0]["kind"] = "flux_capacitor"
    with pytest.raises(ValueError, match="not supported"):
        hcs.import_hardware_config(document)


def test_import_rejects_duplicate_controller_addresses(seeded):
    document = hcs.export_hardware_config()
    document["controllers"][0]["address"] = "same"
    document["controllers"][1]["address"] = "same"
    with pytest.raises(ValueError, match="Duplicate controller address"):
        hcs.import_hardware_config(document)


def test_v1_import_rejects_invalid_uid(seeded):
    document = {
        "version": 1,
        "bricklets": [
            {"brickletNumber": 1, "uid": "BAD_UID!", "type": "Servo Bricklet"}
        ],
        "motors": [],
    }
    with pytest.raises(ValueError, match="invalid format"):
        hcs.import_hardware_config(document)


@pytest.mark.parametrize(
    "uid",
    ["A", "SRV123", "Servo1", "abcdef", "zzzzzz", "9"],
)
def test_validate_uid_accepts_base58_uids(uid):
    assert hcs.validate_uid(uid) == uid


@pytest.mark.parametrize("uid", ["", "   "])
def test_validate_uid_accepts_the_empty_string_as_not_configured(uid):
    assert hcs.validate_uid(uid) == ""


@pytest.mark.parametrize(
    "uid",
    [
        "E2E001",  # '0' - the UID that crash-looped the motor node
        "SERVO1",  # 'O'
        "ABCI12",  # 'I'
        "abcl12",  # 'l'
        "SERVO12",  # more than six characters
        "BAD_UID!",  # not alphanumeric at all
        "AB CD",  # inner whitespace
    ],
)
def test_validate_uid_rejects_non_base58_uids(uid):
    with pytest.raises(ValueError, match="invalid format"):
        hcs.validate_uid(uid)


def test_validate_uid_names_the_value_and_the_forbidden_characters():
    with pytest.raises(ValueError) as raised:
        hcs.validate_uid("E2E001", "Bricklet UID")

    message = str(raised.value)
    assert "Bricklet UID has invalid format 'E2E001'" in message
    for character in hcs.UID_FORBIDDEN_CHARACTERS:
        assert f"'{character}'" in message


def test_validate_uid_rejects_non_string_values():
    with pytest.raises(ValueError, match="must be a string"):
        hcs.validate_uid(1234)


def test_v1_import_validator_delegates_to_the_shared_uid_helper(seeded, monkeypatch):
    calls = []
    real_validate_uid = hcs.validate_uid

    def spy(value, field="uid"):
        calls.append(field)
        return real_validate_uid(value, field)

    monkeypatch.setattr(hcs, "validate_uid", spy)
    hcs.validate_hardware_config(
        {
            "version": 1,
            "bricklets": [
                {"brickletNumber": 1, "uid": "SRV111", "type": "Servo Bricklet"}
            ],
            "motors": [],
        }
    )

    assert calls == ["bricklets[0].uid"]


def test_import_rejects_unknown_motor(seeded):
    document = hcs.export_hardware_config()
    document["motors"].append({"name": "missing", "controllerNumber": 1, "channel": 0})
    with pytest.raises(ValueError, match="Unknown motor"):
        hcs.import_hardware_config(document)


def test_validate_accepts_v2_snake_case_aliases(seeded):
    document = {
        "version": 2,
        "controllers": [
            {
                "kind": "tinkerforge_bricklet",
                "address": "SNK001",
                "number": 1,
                "supply_voltage": 7.5,
            }
        ],
        "motors": [
            {
                "name": "elbow_left",
                "pulse_width_min": 800,
                "controller_number": 1,
                "channel": 8,
            }
        ],
    }
    validated = hcs.validate_hardware_config(document)
    assert validated["controllers"][0]["supply_voltage"] == 7.5
    assert validated["motors"][0]["settings"]["pulse_width_min"] == 800


def test_controller_model_rejects_unregistered_kind(app_ctx):
    with pytest.raises(ValueError, match="Unsupported controller kind"):
        Controller(kind="unsupported", number=99)


def test_controller_model_validates_device_type_by_kind(app_ctx):
    with pytest.raises(ValueError, match="Unsupported Tinkerforge device type"):
        Controller(
            kind="tinkerforge_bricklet",
            device_type="Mystery Bricklet",
            number=98,
        )
    with pytest.raises(ValueError, match="must be None"):
        Controller(
            kind="robstride_can",
            device_type="Servo Bricklet",
            number=99,
        )
