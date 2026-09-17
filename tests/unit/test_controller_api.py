from app.app import db
from model.controller_model import Controller


def test_controller_endpoint_uses_generic_schema(app):
    with app.test_client() as client:
        response = client.get("/controller")

    assert response.status_code == 200
    controller = response.get_json()["controllers"][0]
    assert set(controller) == {
        "kind",
        "deviceType",
        "address",
        "number",
        "supplyVoltage",
    }


def test_controller_address_update(app):
    with app.test_client() as client:
        response = client.put("/controller/1", json={"address": "NEW001"})

    assert response.status_code == 200
    assert response.get_json()["address"] == "NEW001"
    with app.app_context():
        assert Controller.query.filter_by(number=1).one().address == "NEW001"


def test_bricklet_alias_keeps_legacy_response(app):
    with app.test_client() as client:
        response = client.get("/bricklet")

    assert response.status_code == 200
    first = response.get_json()["bricklets"][0]
    assert set(first) == {"brickletNumber", "uid", "type"}


def test_pib5edu_device_types_do_not_depend_on_controller_number(app):
    expected_types = {
        1: "Servo Bricklet",
        2: "Servo Bricklet",
        3: "Servo Bricklet",
        4: "Servo Bricklet",
        5: "Solid State Relay Bricklet",
        6: "RGB LED Button Bricklet",
        7: "RGB LED Button Bricklet",
        8: "RGB LED Button Bricklet",
    }
    with app.app_context():
        for number, device_type in expected_types.items():
            controller = Controller.query.filter_by(number=number).one_or_none()
            if controller is None:
                controller = Controller(
                    number=number,
                    kind="tinkerforge_bricklet",
                )
                db.session.add(controller)
            controller.device_type = device_type
        db.session.commit()

    with app.test_client() as client:
        controllers = client.get("/controller").get_json()["controllers"]
        bricklets = client.get("/bricklet").get_json()["bricklets"]

    assert {
        item["number"]: item["deviceType"] for item in controllers
    } == expected_types
    assert {
        item["brickletNumber"]: item["type"] for item in bricklets
    } == expected_types


def test_motor_endpoint_uses_controller_and_channel(app):
    with app.test_client() as client:
        motor = client.get("/motor/elbow_left").get_json()
        assert motor["controller"]["number"] == 3
        assert motor["channel"] == 8
        assert "brickletPins" not in motor

        motor["currentLimit"] = 1.5
        response = client.put("/motor/elbow_left", json=motor)

    assert response.status_code == 200
    assert response.get_json()["currentLimit"] == 1.5
