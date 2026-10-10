"""A motor controller update without a controller answers 400, not 500 (PR-1978).

Measured on develop ``aec4deae`` with the seeded pib5edu profile:

    PUT /motor/elbow_left/controller {"controller": {}, "channel": 2}
        -> 500 {"error": "an unknown error occured."}

``motor_service.set_motor_controller`` raised a bare ``ValueError`` that the app's
handler list does not map. ``PUT /motor/<name>`` calls the same service function, but
its schema already refuses a nested controller without ``number`` (400) before the
service runs.
"""

import pytest

from app.app import db
from model.motor_model import Motor
from service import motor_service

MOTOR = "elbow_left"
REASON = "Controller requires a number or address"


def _stored_controller_and_channel(app):
    with app.app_context():
        motor = db.session.query(Motor).filter_by(name=MOTOR).one()
        return motor.controller.number, motor.channel


@pytest.mark.parametrize(
    "controller", [{}, {"number": None, "address": None}], ids=["empty", "nulls"]
)
def test_update_motor_controller_without_controller_answers_bad_request(
    app, controller
):
    before = _stored_controller_and_channel(app)

    with app.test_client() as client:
        response = client.put(
            f"/motor/{MOTOR}/controller",
            json={"controller": controller, "channel": 0},
        )

    assert response.status_code == 400, response.get_data(as_text=True)
    assert response.get_json() == {"error": REASON}
    assert _stored_controller_and_channel(app) == before


def test_update_motor_controller_still_updates(app):
    with app.test_client() as client:
        response = client.put(
            f"/motor/{MOTOR}/controller",
            json={"controller": {"number": 4}, "channel": 1},
        )

    assert response.status_code == 200, response.get_data(as_text=True)
    body = response.get_json()
    assert set(body) == {"name", "controller", "channel"}
    assert body["name"] == MOTOR
    assert body["channel"] == 1
    assert body["controller"]["number"] == 4


def test_update_motor_without_controller_answers_bad_request(app):
    """The schema refuses this payload before the service runs - still a 400."""
    with app.test_client() as client:
        motor = client.get(f"/motor/{MOTOR}").get_json()
        motor["controller"] = {}
        response = client.put(f"/motor/{MOTOR}", json=motor)

    assert response.status_code == 400, response.get_data(as_text=True)
    assert "error" in response.get_json()


def test_update_motor_still_updates(app):
    with app.test_client() as client:
        motor = client.get(f"/motor/{MOTOR}").get_json()
        motor["channel"] = 1
        motor["velocity"] = 12345
        response = client.put(f"/motor/{MOTOR}", json=motor)

    assert response.status_code == 200, response.get_data(as_text=True)
    assert response.get_json() == motor


def test_service_raises_named_error(app):
    """The service names the condition, so the next route cannot miss the mapping."""
    with app.app_context():
        with pytest.raises(motor_service.MotorValidationError, match=REASON):
            motor_service.set_motor_controller(MOTOR, {}, 0)
