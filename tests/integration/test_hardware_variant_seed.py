"""Fresh-database seed coverage for the implemented hardware variants."""

import pytest
from click.testing import CliRunner

from app.app import db
from commands import seed_db
from model.controller_model import Controller
from model.motor_model import Motor


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
        for motor_name, expected_location in relocated_motors.items():
            motor = Motor.query.filter_by(name=motor_name).one()
            assert (motor.controller.number, motor.channel) == expected_location


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
