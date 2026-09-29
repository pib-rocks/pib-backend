def _mapping_payload(bricklet_number):
    return {
        "buttonProgramUpdates": [
            {"brickletNumber": bricklet_number, "programNumber": None}
        ]
    }


def test_unknown_controller_returns_not_found(app):
    with app.test_client() as client:
        response = client.put("/button-programs", json=_mapping_payload(99))

    assert response.status_code == 404
    assert response.get_json() == {
        "error": "Entity not found. Please check your path parameter."
    }


def test_controller_without_button_program_returns_unprocessable_entity(app):
    with app.test_client() as client:
        response = client.put("/button-programs", json=_mapping_payload(5))

    assert response.status_code == 422
    assert response.get_json() == {
        "error": "No button program exists for buttonProgramUpdates[].brickletNumber=5."
    }
