"""Fresh-database seed coverage for the implemented hardware variants."""

import json

import pytest
from click.testing import CliRunner

from app.app import db
from commands import LEGACY_CEREBRA_TOGGLE_PROGRAM_NUMBER, seed_db
from model.button_program_model import ButtonProgram
from model.controller_model import Controller
from model.motor_model import Motor
from model.program_model import Program
from model.system_property_model import SystemProperty
from seed_profiles import get_profile
from service.system_property_service import (
    HARDWARE_VARIANT_KEY,
    MICROPHONE_DESIRED_STATE_KEY,
)


@pytest.mark.parametrize(
    ("variant", "controller_count", "relocated_motors"),
    [
        (
            "pib4edu",
            7,
            {
                "shoulder_vertical_right": (2, 1),
                "shoulder_vertical_left": (2, 9),
                "elbow_left": (3, 8),
                "elbow_right": (1, 8),
            },
        ),
        (
            "pib5edu",
            8,
            {
                "shoulder_vertical_right": (4, 0),
                "shoulder_vertical_left": (4, 1),
                "elbow_left": (4, 2),
                "elbow_right": (4, 3),
            },
        ),
    ],
)
def test_seed_db_uses_selected_hardware_profile(
    app,
    monkeypatch: pytest.MonkeyPatch,
    variant: str,
    controller_count: int,
    relocated_motors: dict[str, tuple[int, int]],
):
    monkeypatch.setenv("PIB_HARDWARE_VARIANT", variant)

    with app.app_context():
        db.session.remove()
        db.drop_all()
        db.create_all()

        result = CliRunner().invoke(seed_db, [])

        assert result.exit_code == 0, result.output
        assert variant in result.output
        assert Controller.query.count() == controller_count
        assert Motor.query.count() == 26
        stored_variant = db.session.get(SystemProperty, HARDWARE_VARIANT_KEY)
        assert (stored_variant.value, stored_variant.source) == (
            variant,
            "environment",
        )
        desired_state = db.session.get(SystemProperty, MICROPHONE_DESIRED_STATE_KEY)
        desired_document = json.loads(desired_state.value)
        assert desired_document["parameters"] == dict(
            get_profile(variant).microphone_tuning
        )
        assert desired_document["revision"] == 1
        for motor_name, expected_location in relocated_motors.items():
            motor = Motor.query.filter_by(name=motor_name).one()
            assert (motor.controller.number, motor.channel) == expected_location
        # a fresh seed binds no program to any LED button
        assert ButtonProgram.query.count() == 3
        assert {item.program_id for item in ButtonProgram.query.all()} == {None}
        assert (
            Program.query.filter_by(
                program_number=LEGACY_CEREBRA_TOGGLE_PROGRAM_NUMBER
            ).first()
            is None
        )


def test_seed_db_unbinds_button_from_legacy_cerebra_toggle_program(app):
    """An already seeded database keeps its rows, but the dead binding goes."""
    with app.app_context():
        legacy = Program(
            name="legacy cerebra toggle",
            code_visual='<xml xmlns="https://developers.google.com/blockly/xml"></xml>',
            program_number=LEGACY_CEREBRA_TOGGLE_PROGRAM_NUMBER,
        )
        db.session.add(legacy)
        db.session.flush()
        buttons = ButtonProgram.query.order_by(ButtonProgram.id).all()
        assert len(buttons) == 3
        buttons[2].program_id = legacy.id
        db.session.commit()
        programs_before = Program.query.count()
        controllers_before = Controller.query.count()

        result = CliRunner().invoke(seed_db, [])

        assert result.exit_code == 0, result.output
        assert "already contains data" in result.output
        assert "Unbound 1 button(s)" in result.output
        assert {item.program_id for item in ButtonProgram.query.all()} == {None}
        assert ButtonProgram.query.count() == 3
        # the program row and everything else is left alone
        assert Program.query.count() == programs_before
        assert Controller.query.count() == controllers_before
        assert (
            Program.query.filter_by(program_number=LEGACY_CEREBRA_TOGGLE_PROGRAM_NUMBER)
            .one()
            .name
            == "legacy cerebra toggle"
        )


def test_seed_db_on_seeded_database_without_legacy_program_is_quiet(app):
    with app.app_context():
        bindings_before = {
            item.controller_id: item.program_id for item in ButtonProgram.query.all()
        }

        result = CliRunner().invoke(seed_db, [])

        assert result.exit_code == 0, result.output
        assert "Unbound" not in result.output
        assert {
            item.controller_id: item.program_id for item in ButtonProgram.query.all()
        } == bindings_before


def test_seed_db_fails_for_variant_without_profile(
    app,
    monkeypatch: pytest.MonkeyPatch,
):
    monkeypatch.setenv("PIB_HARDWARE_VARIANT", "pib5advanced")

    with app.app_context():
        db.session.remove()
        db.drop_all()
        db.create_all()

        result = CliRunner().invoke(seed_db, [])

        assert result.exit_code != 0
        assert "pib5advanced" in str(result.exception)
        assert "pib4edu" in str(result.exception)
        assert "pib5edu" in str(result.exception)
        assert Controller.query.count() == 0
        assert Motor.query.count() == 0
