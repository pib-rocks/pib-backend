"""PC-121: the pib5advanced variant has a seed profile, so its robot can start.

The container runs ``flask db upgrade && flask seed_db && …``. A variant without
a profile raised in ``get_profile`` and the flask container restart-looped, which
is why its System/Hardware-IDs page showed nothing at all - not even the
Tinkerforge parts.

This variant is wired differently from the educational robots: four serial
controllers carry the motors, and the Tinkerforge HAT carries only the relay and
up to three RGB LED buttons. The identifiers of the serial adapters are read
separately, so the profile ships without any address and every controller row is
created without one. What is entered later has to survive the next start, and a
profile without a motor mapping must not read as "delete every motor".
"""

from __future__ import annotations

import pytest
from click.testing import CliRunner

from app.app import db
from commands import _upsert_controllers, _upsert_motors, seed_db
from model.button_program_model import ButtonProgram
from model.controller_model import FEETECH_ST_SERIAL, Controller
from model.motor_model import Motor
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


def test_the_profile_declares_no_motors_yet():
    """Which motor sits on which channel follows the identifiers, not a guess."""
    assert get_profile("pib5advanced").motor_mapping == {}


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
    # No joint is invented: no motors, and therefore empty start poses.
    assert Motor.query.count() == 0
    assert Pose.query.count() == 2


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
