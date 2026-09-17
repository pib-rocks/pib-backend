"""Integration coverage for deliberate hardware profile switching."""

from pathlib import Path

import click
import pytest

from click.testing import CliRunner

from app.app import db
from commands import seed_hardware
from model.button_program_model import ButtonProgram
from model.chat_model import Chat
from model.controller_model import Controller
from model.motor_model import Motor
from model.motor_position_model import MotorPosition
from model.pose_model import Pose
from model.program_model import Program
from model.system_property_model import SystemProperty
from seed_profiles import get_profile
from service.system_property_service import HARDWARE_VARIANT_KEY


def _database_path(app) -> Path:
    # the command backs up the database of the live engine, so look for the backup next to it
    with app.app_context():
        return Path(str(db.engine.url.database)).expanduser().resolve()


def _backup_paths(app) -> list[Path]:
    database_path = _database_path(app)
    return list(database_path.parent.glob(f"{database_path.name}.bak-*"))


def _hardware_state() -> tuple:
    controllers = tuple(
        (
            item.id,
            item.number,
            item.kind,
            item.device_type,
            item.address,
            item.supply_voltage,
        )
        for item in Controller.query.order_by(Controller.id)
    )
    motors = tuple(
        (
            item.id,
            item.name,
            item.controller_id,
            item.channel,
            item.pulse_width_min,
            item.pulse_width_max,
            item.rotation_range_min,
            item.rotation_range_max,
            item.velocity,
            item.acceleration,
            item.deceleration,
            item.period,
            item.turned_on,
            item.visible,
            item.invert,
            item.current_limit,
            item.torque_limit,
        )
        for item in Motor.query.order_by(Motor.id)
    )
    buttons = tuple(
        (item.id, item.controller_id, item.program_id)
        for item in ButtonProgram.query.order_by(ButtonProgram.id)
    )
    variant = db.session.get(SystemProperty, HARDWARE_VARIANT_KEY)
    return controllers, motors, buttons, (variant.value, variant.source)


def _protected_state() -> tuple:
    counts = (Pose.query.count(), Program.query.count(), Chat.query.count())
    positions = tuple(
        (item.id, item.position, item.motor_name, item.pose_id)
        for item in MotorPosition.query.order_by(MotorPosition.id)
    )
    return counts, positions


def _invoke(variant: str, confirmation: str | None = None):
    arguments = ["--variant", variant, "--force"]
    input_text = None if confirmation is None else f"{confirmation}\n"
    return CliRunner().invoke(seed_hardware, arguments, input=input_text)


def test_seed_hardware_without_force_changes_nothing_and_makes_no_backup(app):
    with app.app_context():
        before = _hardware_state()
        backups_before = set(_backup_paths(app))

        result = CliRunner().invoke(seed_hardware, ["--variant", "pib4edu"])

        assert result.exit_code != 0
        assert "--force" in result.output
        assert _hardware_state() == before
        assert set(_backup_paths(app)) == backups_before


def test_seed_hardware_wrong_confirmation_changes_nothing_and_makes_no_backup(app):
    with app.app_context():
        before = _hardware_state()
        backups_before = set(_backup_paths(app))

        result = _invoke("pib4edu", "PIB4EDU")

        assert result.exit_code != 0
        assert "did not match" in result.output
        assert _hardware_state() == before
        assert set(_backup_paths(app)) == backups_before


def test_seed_hardware_switches_profile_and_preserves_protected_rows(app):
    profile = get_profile("pib4edu")
    expected_controllers = {
        entry.number: (entry.kind, entry.device_type, entry.supply_voltage)
        for entry in profile.controllers
    }
    relocated = {
        "shoulder_vertical_right": (2, 1),
        "shoulder_vertical_left": (2, 9),
        "elbow_left": (3, 8),
        "elbow_right": (1, 8),
    }

    with app.app_context():
        protected_before = _protected_state()
        backups_before = set(_backup_paths(app))

        result = _invoke("pib4edu", "pib4edu")

        assert result.exit_code == 0, result.output
        assert {
            item.number: (item.kind, item.device_type, item.supply_voltage)
            for item in Controller.query.all()
        } == expected_controllers
        assert {
            name: (
                Motor.query.filter_by(name=name).one().controller.number,
                Motor.query.filter_by(name=name).one().channel,
            )
            for name in relocated
        } == relocated
        assert {item.controller.number for item in ButtonProgram.query.all()} == set(
            profile.rgb_button_controller_ids
        )
        fullscreen = Program.query.filter_by(name="toggle_cerebra_fullscreen").one()
        assert (
            ButtonProgram.query.filter_by(program_id=fullscreen.id)
            .one()
            .controller.number
            == profile.rgb_button_controller_ids[2]
        )
        assert _protected_state() == protected_before
        new_backups = set(_backup_paths(app)) - backups_before
        assert len(new_backups) == 1
        assert next(iter(new_backups)).is_file()
        stored = db.session.get(SystemProperty, HARDWARE_VARIANT_KEY)
        assert (stored.value, stored.source) == ("pib4edu", "command")
        assert "Protected counts unchanged: pose=yes, program=yes, chat=yes" in (
            result.output
        )


def test_seed_hardware_switching_back_keeps_controller_address(app):
    with app.app_context():
        controller = Controller.query.filter_by(number=1).one()
        controller.address = "UID123"
        db.session.commit()

        to_pib4 = _invoke("pib4edu", "pib4edu")
        to_pib5 = _invoke("pib5edu", "pib5edu")
        back_to_pib4 = _invoke("pib4edu", "pib4edu")

        assert to_pib4.exit_code == 0, to_pib4.output
        assert to_pib5.exit_code == 0, to_pib5.output
        assert back_to_pib4.exit_code == 0, back_to_pib4.output
        controller = Controller.query.filter_by(number=1).one()
        assert controller.address == "UID123"
        assert Controller.query.count() == 7


def test_seed_hardware_unimplemented_variant_changes_nothing(app):
    with app.app_context():
        before = _hardware_state()
        backups_before = set(_backup_paths(app))

        result = _invoke("pib5museum")

        assert result.exit_code != 0
        assert "pib5museum" in result.output
        assert "pib4edu" in result.output
        assert "pib5edu" in result.output
        assert _hardware_state() == before
        assert set(_backup_paths(app)) == backups_before


def test_seed_hardware_unknown_variant_is_rejected_before_profile_lookup(app):
    with app.app_context():
        before = _hardware_state()
        backups_before = set(_backup_paths(app))

        result = _invoke("pib9000")

        assert result.exit_code != 0
        assert "Invalid value for '--variant'" in result.output
        assert "pib4edu" in result.output
        assert "pib5edu" in result.output
        assert _hardware_state() == before
        assert set(_backup_paths(app)) == backups_before


def test_seed_hardware_rejects_unbackupable_databases(app):
    """the guard covers a non-SQLite URL and an in-memory SQLite URL"""
    from sqlalchemy.engine import make_url

    from commands import _backup_sqlite_database, _file_backed_sqlite_url

    with app.app_context():
        before = _hardware_state()
        backups_before = set(_backup_paths(app))

        with pytest.raises(click.ClickException) as postgres_error:
            _file_backed_sqlite_url(make_url("postgresql://localhost/pib"))
        assert "file-backed SQLite" in str(postgres_error.value)

        with pytest.raises(click.ClickException) as memory_error:
            _file_backed_sqlite_url(make_url("sqlite://"))
        assert "file-backed SQLite" in str(memory_error.value)

        with pytest.raises(click.ClickException):
            _backup_sqlite_database(make_url("postgresql://localhost/pib"))

        assert _hardware_state() == before
        assert set(_backup_paths(app)) == backups_before


def test_seed_hardware_preserves_existing_motor_parameters(app):
    with app.app_context():
        motor = Motor.query.filter_by(name="elbow_left").one()
        motor.velocity = 424242
        motor.current_limit = 1.75
        db.session.commit()

        result = _invoke("pib4edu", "pib4edu")

        assert result.exit_code == 0, result.output
        motor = Motor.query.filter_by(name="elbow_left").one()
        assert motor.velocity == 424242
        assert motor.current_limit == 1.75
