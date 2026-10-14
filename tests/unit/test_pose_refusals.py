"""A protected pose answers a refusal, not a server error (PR-1974).

Measured on the robot during the full E2E run - every one of these used to be a bare
``ValueError``, which the app's handler list does not map, so all three posed as
``500 {"error": "an unknown error occured."}``:

    DELETE /pose/<calibration id>                        -> 500
    PATCH  /pose/<calibration id>            {"name": …} -> 500
    PATCH  /pose/<calibration id>/motor-positions {…}    -> 500

A caller cannot tell "you may not change this pose" from "the server is broken" when
both arrive as 500, and the live E2E suite failed on exactly that ambiguity.
"""

import pytest

from app.app import db
from default_pose_constants import CALIBRATION_POSE_NAME, STARTUP_POSE_NAME
from model.pose_model import Pose
from service import pose_service

PROTECTED_POSES = [STARTUP_POSE_NAME, CALIBRATION_POSE_NAME]


def _pose_id(app, name):
    with app.app_context():
        return db.session.query(Pose).filter_by(name=name).one().pose_id


def _motor_positions_of(app, name):
    """The stored positions as the API wants them back (camelCase body)."""
    with app.app_context():
        pose = db.session.query(Pose).filter_by(name=name).one()
        return [
            {"motorName": position.motor_name, "position": position.position}
            for position in pose.motor_positions
        ]


@pytest.mark.parametrize("name", PROTECTED_POSES)
def test_protected_pose_delete_answers_conflict(app, name):
    pose_id = _pose_id(app, name)

    with app.test_client() as client:
        response = client.delete(f"/pose/{pose_id}")

    assert response.status_code == 409, response.get_data(as_text=True)
    assert name in response.get_json()["error"]
    with app.app_context():
        assert db.session.query(Pose).filter_by(pose_id=pose_id).one() is not None


@pytest.mark.parametrize("name", PROTECTED_POSES)
def test_protected_pose_rename_answers_conflict(app, name):
    pose_id = _pose_id(app, name)

    with app.test_client() as client:
        response = client.patch(f"/pose/{pose_id}", json={"name": "Renamed by test"})

    assert response.status_code == 409, response.get_data(as_text=True)
    assert name in response.get_json()["error"]
    with app.app_context():
        assert db.session.query(Pose).filter_by(pose_id=pose_id).one().name == name


def test_protected_pose_position_update_answers_conflict(app):
    pose_id = _pose_id(app, CALIBRATION_POSE_NAME)
    positions = _motor_positions_of(app, CALIBRATION_POSE_NAME)

    with app.test_client() as client:
        response = client.patch(
            f"/pose/{pose_id}/motor-positions", json={"motorPositions": positions}
        )

    assert response.status_code == 409, response.get_data(as_text=True)
    assert CALIBRATION_POSE_NAME in response.get_json()["error"]


def test_startup_pose_positions_may_still_be_updated(app):
    """The Startup/Resting pose is the one protected pose the API lets you tune."""
    pose_id = _pose_id(app, STARTUP_POSE_NAME)
    positions = _motor_positions_of(app, STARTUP_POSE_NAME)

    with app.test_client() as client:
        response = client.patch(
            f"/pose/{pose_id}/motor-positions", json={"motorPositions": positions}
        )

    assert response.status_code == 200, response.get_data(as_text=True)


def test_position_update_with_wrong_count_answers_bad_request(app):
    """A count mismatch is a bad request - and it names the mismatch."""
    with app.app_context():
        pose = pose_service.create_pose(
            {
                "name": "Pose with one position",
                "motor_positions": [{"motor_name": "turn_head_motor", "position": 0}],
            }
        )
        db.session.commit()
        pose_id = pose.pose_id

    try:
        with app.test_client() as client:
            response = client.patch(
                f"/pose/{pose_id}/motor-positions",
                json={
                    "motorPositions": [
                        {"motorName": "turn_head_motor", "position": 0},
                        {"motorName": "tilt_forward_motor", "position": 0},
                    ]
                },
            )
    finally:
        with app.app_context():
            db.session.query(Pose).filter_by(pose_id=pose_id).delete()
            db.session.commit()

    assert response.status_code == 400, response.get_data(as_text=True)
    assert "motor positions" in response.get_json()["error"]


def test_service_raises_named_errors(app):
    """The service names the conditions, so the mapping cannot be missed silently."""
    pose_id = _pose_id(app, CALIBRATION_POSE_NAME)

    with app.app_context():
        with pytest.raises(pose_service.PoseRefusedError):
            pose_service.delete_pose(pose_id)
        with pytest.raises(pose_service.PoseRefusedError):
            pose_service.rename_pose(pose_id, {"name": "Renamed by test"})
        with pytest.raises(pose_service.PoseRefusedError):
            pose_service.update_motor_positions_of_pose(
                pose_id,
                {"motor_positions": _motor_positions_of(app, CALIBRATION_POSE_NAME)},
            )

        assert db.session.query(Pose).filter_by(pose_id=pose_id).one() is not None


def test_deletable_pose_still_deletes(app):
    with app.app_context():
        pose = pose_service.create_pose(
            {
                "name": "Temp Pose PR-1974",
                "motor_positions": [{"motor_name": "turn_head_motor", "position": 0}],
            }
        )
        db.session.commit()
        pose_id = pose.pose_id

    with app.test_client() as client:
        response = client.delete(f"/pose/{pose_id}")

    assert response.status_code == 204, response.get_data(as_text=True)
    with app.app_context():
        assert db.session.query(Pose).filter_by(pose_id=pose_id).first() is None
