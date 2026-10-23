"""GET /system/motion-model exposes DB-backed body context for Direct turns."""

from model.motor_model import Motor

_UNITS_PREFIX = (
    "Body (centidegrees: 1 = 0.01 degree, 0 = joint zero, " "+/-9000 = +/-90 degrees)."
)


def test_motion_model_endpoint_lists_visible_motors_in_prompt(client):
    response = client.get("/system/motion-model")
    assert response.status_code == 200
    body = response.get_json()
    prompt = body["prompt"]
    assert prompt.startswith(_UNITS_PREFIX)
    assert body["schemaVersion"] == 1
    assert "chains" in body
    assert "postures" in body

    for motor in Motor.query.filter_by(visible=True).order_by(Motor.name):
        assert motor.name in prompt
