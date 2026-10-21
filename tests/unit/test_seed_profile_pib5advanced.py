"""PC-121 & PC-125: the pib5advanced variant has a seed profile and serial motor mapping.

The container runs ``flask db upgrade && flask seed_db && …``. A variant without
a profile raised in ``get_profile`` and the flask container restart-looped, which
is why its System/Hardware-IDs page showed nothing at all - not even the
Tinkerforge parts.

This variant is wired differently from the educational robots: four serial
controllers carry the 26 Feetech STS motors, and the Tinkerforge HAT carries only
the relay and up to three RGB LED buttons. The identifiers of the serial adapters
are read separately, so the profile ships without any address and every controller
row is created without one. What is entered later has to survive the next start,
and an earlier motorless database is backfilled with the 26 motors on startup.
"""

from __future__ import annotations

import pytest
from click.testing import CliRunner

from app.app import db
from commands import _upsert_controllers, _upsert_motors, seed_db
from model.button_program_model import ButtonProgram
from model.controller_model import FEETECH_ST_SERIAL, Controller
from model.motor_model import Motor
from model.motor_position_model import MotorPosition
from model.pose_model import Pose
from seed_profiles import PROFILES, get_profile
from service.system_property_service import HARDWARE_VARIANT_KEY, get_property_value


@pytest.fixture()
def seeded_as_pib5advanced(app_ctx, monkeypatch):
    """Seed an empty database as the advanced variant, the way the container does.

    The shared ``app`` fixture seeds pib5edu, so this starts from an empty schema:
    the point of the change is the very first start of a fresh robot.
    """
    db.drop_all()
    db.create_all()
    monkeypatch.setenv("PIB_HARDWARE_VARIANT", "pib5advanced")
    monkeypatch.setattr("commands._create_chat_data_and_assistant", lambda: None)
    result = CliRunner().invoke(seed_db)
    assert result.exit_code == 0, result.output
    assert "pib5advanced" in result.output
    return result


def test_the_variant_is_implemented():
    """A variant name the property service accepts must have a profile to seed."""
    from service.system_property_service import ALLOWED_HARDWARE_VARIANTS

    assert "pib5advanced" in ALLOWED_HARDWARE_VARIANTS
    assert "pib5advanced" in PROFILES


def test_the_profile_matches_the_electronics():
    profile = get_profile("pib5advanced")

    serial = [c for c in profile.controllers if c.kind == FEETECH_ST_SERIAL]
    bricklets = [c for c in profile.controllers if c.kind == "tinkerforge_bricklet"]
    device_types = sorted(str(c.device_type) for c in bricklets)

    assert [c.number for c in serial] == [1, 2, 3, 4]
    # The motors are on the serial bus, so no servo bricklet exists here.
    assert "Servo Bricklet" not in device_types
    assert device_types == [
        "RGB LED Button Bricklet",
        "RGB LED Button Bricklet",
        "RGB LED Button Bricklet",
        "Solid State Relay Bricklet",
    ]
    assert profile.rgb_button_controller_ids == (6, 7, 8)


def test_every_controller_starts_without_an_identifier():
    """The serial identifiers are read separately and entered later."""
    profile = get_profile("pib5advanced")

    assert all(controller.address is None for controller in profile.controllers)


def test_the_profile_maps_all_26_motors_onto_the_four_serial_controllers():
    """Each of the 26 joints maps to one of controllers 1..4 and its STS bus ID."""
    profile = get_profile("pib5advanced")

    assert len(profile.motor_mapping) == 26
    assert set(profile.motor_mapping) == set(get_profile("pib5edu").motor_mapping)
    controller_numbers = {num for num, _channel in profile.motor_mapping.values()}
    assert controller_numbers == {1, 2, 3, 4}
    assert profile.motor_mapping["turn_head_motor"] == (1, 51)
    assert profile.motor_mapping["elbow_right"] == (2, 18)
    assert profile.motor_mapping["wrist_right"] == (2, 16)
    assert profile.motor_mapping["tilt_forward_motor"] == (3, 50)
    assert profile.motor_mapping["elbow_left"] == (4, 38)
    assert profile.motor_mapping["wrist_left"] == (4, 36)


def test_seeding_the_advanced_variant_brings_the_robot_up(seeded_as_pib5advanced):
    controllers = {
        controller.number: controller
        for controller in Controller.query.order_by(Controller.number)
    }

    assert sorted(controllers) == [1, 2, 3, 4, 5, 6, 7, 8]
    assert controllers[1].kind == FEETECH_ST_SERIAL
    assert controllers[1].address is None
    assert controllers[5].device_type == "Solid State Relay Bricklet"
    assert controllers[6].device_type == "RGB LED Button Bricklet"

    # The three LED buttons exist with an empty program, so the page has rows.
    assert ButtonProgram.query.count() == 3
    # The System page reports the variant that was seeded.
    assert get_property_value(HARDWARE_VARIANT_KEY) == "pib5advanced"
    assert Motor.query.count() == 26
    assert Pose.query.count() == 2
    assert MotorPosition.query.count() == 52


def test_backfills_motors_when_an_earlier_motorless_advanced_db_starts(
    seeded_as_pib5advanced, monkeypatch
):
    """A DB seeded under PC-121 (0 motors, stored adapter IDs) gets its 26 motors on start."""
    controller = Controller.query.filter_by(number=1).one()
    controller.address = "5B3D042723"
    MotorPosition.query.delete()
    Motor.query.delete()
    db.session.commit()
    assert Motor.query.count() == 0

    monkeypatch.setattr("commands._reconcile_model_catalogue", lambda: None)
    result = CliRunner().invoke(seed_db)
    assert result.exit_code == 0, result.output

    assert Motor.query.count() == 26
    assert Controller.query.filter_by(number=1).one().address == "5B3D042723"
    head = Motor.query.filter_by(name="turn_head_motor").one()
    assert head.controller.number == 1
    assert head.channel == 51
    assert MotorPosition.query.count() == 52


def test_an_identifier_entered_later_survives_the_next_start(seeded_as_pib5advanced):
    """What is read and entered by hand must not be overwritten by a re-seed."""
    controller = Controller.query.filter_by(number=1).one()
    controller.address = "/dev/ttyUSB0"
    db.session.commit()

    _upsert_controllers(get_profile("pib5advanced"))
    db.session.commit()

    assert Controller.query.filter_by(number=1).one().address == "/dev/ttyUSB0"


def test_a_profile_without_motors_does_not_delete_the_motors_that_are_there(app_ctx):
    """`~Motor.name.in_(())` matches every row: an empty mapping is not a delete."""
    educational = get_profile("pib5edu")
    _upsert_controllers(educational)
    _upsert_motors(educational)
    db.session.commit()
    present = Motor.query.count()
    assert present > 0

    empty_mapping = educational.__class__(
        variant="pib5advanced",
        description="no motors declared",
        controllers=educational.controllers,
        motor_mapping={},
        rgb_button_controller_ids=educational.rgb_button_controller_ids,
        motor_parameter_defaults=educational.motor_parameter_defaults,
        motor_parameter_deviations={},
        microphone_tuning=educational.microphone_tuning,
    )

    stats, _warnings = _upsert_motors(empty_mapping)
    db.session.commit()

    assert stats["deleted"] == 0
    assert Motor.query.count() == present
