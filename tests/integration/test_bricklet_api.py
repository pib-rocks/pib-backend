"""Integration tests for the Bricklet write path (PR-1796).

`PUT /bricklet/<n>` is what the Hardware-IDs UI calls. It used to accept any
string, which is how the non-Base58 UID 'E2E001' reached the database and
crash-looped the motor node.
"""

from __future__ import annotations

import pytest

from app.app import db
from model.controller_model import Controller


@pytest.fixture()
def bricklet_one(app):
    with app.app_context():
        controller = Controller.query.filter_by(number=1).one()
        controller.address = "SRV111"
        db.session.commit()
    return "SRV111"


def _stored_address(app) -> str | None:
    with app.app_context():
        return Controller.query.filter_by(number=1).one().address


def test_put_bricklet_rejects_a_non_base58_uid(client, app, bricklet_one):
    response = client.put("/bricklet/1", json={"uid": "E2E001"})

    assert response.status_code == 400
    error = response.get_json()["error"]
    assert "E2E001" in error
    assert "invalid format" in error
    assert _stored_address(app) == bricklet_one


@pytest.mark.parametrize("uid", ["SERVO1", "ABCI12", "abcl12", "SERVO12", "bad uid!"])
def test_put_bricklet_rejects_every_flavour_of_invalid_uid(
    client, app, bricklet_one, uid
):
    response = client.put("/bricklet/1", json={"uid": uid})

    assert response.status_code == 400
    assert _stored_address(app) == bricklet_one


def test_put_bricklet_accepts_a_valid_uid(client, app, bricklet_one):
    response = client.put("/bricklet/1", json={"uid": "Servo1"})

    assert response.status_code == 200
    assert response.get_json()["uid"] == "Servo1"
    assert _stored_address(app) == "Servo1"


def test_put_bricklet_accepts_an_empty_uid_to_clear_the_assignment(
    client, app, bricklet_one
):
    response = client.put("/bricklet/1", json={"uid": ""})

    assert response.status_code == 200
    assert response.get_json()["uid"] == ""
    assert not _stored_address(app)


def test_put_bricklet_rejects_a_non_string_uid(client, app, bricklet_one):
    response = client.put("/bricklet/1", json={"uid": 123})

    assert response.status_code == 400
    assert "must be a string" in response.get_json()["error"]
    assert _stored_address(app) == bricklet_one
