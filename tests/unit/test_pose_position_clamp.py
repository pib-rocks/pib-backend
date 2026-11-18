"""A saved pose may only carry positions the robot can play back.

A servo reports its position with its own tolerance, so a recorded joint can land a few
centidegrees beyond the configured range - measured on .168, a pose captured
``shoulder_vertical_left = -9026`` and ``shoulder_vertical_right = -9009``.

Cerebra publishes stored centidegrees unchanged, so such a pose still runs there. A
Blockly program does not: the generated ``apply_pose()`` converts centidegrees to degrees
and calls ``pib_sdk.Write.move()``, whose ``_degrees_to_radians`` refuses anything outside
[-90, 90] with ``ValueError: position_deg must be between -90 and 90 (got -90.26)``. The
program ends in a traceback before a trajectory is sent, so the robot stays in the previous
posture - which reads as "the new pose inherited the old angles".

Saving therefore clamps to the motor's own range, intersected with the playable +-90 deg.
"""

import logging

import pytest

from app.app import db
from model.motor_model import Motor
from model.pose_model import Pose
from service import pose_service


def _store_pose(app, name, positions):
    with app.app_context():
        pose = pose_service.create_pose(
            {
                "name": name,
                "motor_positions": [
                    {"motor_name": motor_name, "position": position}
                    for motor_name, position in positions
                ],
            }
        )
        db.session.commit()
        return pose.pose_id


def _stored_positions(app, pose_id):
    with app.app_context():
        pose = db.session.query(Pose).filter_by(pose_id=pose_id).one()
        return {mp.motor_name: mp.position for mp in pose.motor_positions}


def _drop_pose(app, pose_id):
    with app.app_context():
        db.session.query(Pose).filter_by(pose_id=pose_id).delete()
        db.session.commit()


def test_motor_ranges_used_by_this_test_are_known(app):
    """The expectations below are only meaningful against the seeded profiles."""
    with app.app_context():
        ranges = {
            motor.name: (motor.rotation_range_min, motor.rotation_range_max)
            for motor in db.session.query(Motor).all()
        }
    assert ranges["turn_head_motor"] == (-9000, 9000)
    assert ranges["tilt_forward_motor"] == (-4500, 4500)


def test_new_pose_clamps_positions_to_the_motor_range(app):
    pose_id = _store_pose(
        app,
        "Clamp Test New Pose",
        [("turn_head_motor", -9500), ("tilt_forward_motor", -6000)],
    )
    try:
        stored = _stored_positions(app, pose_id)
        assert stored["turn_head_motor"] == -9000
        # The tilt motor has its own, narrower range than the playable +-90 degrees.
        assert stored["tilt_forward_motor"] == -4500
    finally:
        _drop_pose(app, pose_id)


def test_position_inside_the_range_is_kept(app):
    pose_id = _store_pose(
        app,
        "Clamp Test In Range",
        [("turn_head_motor", -466), ("tilt_forward_motor", 773)],
    )
    try:
        stored = _stored_positions(app, pose_id)
        assert stored["turn_head_motor"] == -466
        assert stored["tilt_forward_motor"] == 773
    finally:
        _drop_pose(app, pose_id)


def test_clamping_is_logged(app, caplog):
    with caplog.at_level(logging.WARNING, logger="service.pose_service"):
        pose_id = _store_pose(app, "Clamp Test Logging", [("turn_head_motor", -9026)])
    try:
        assert any(
            "clamped pose position for turn_head_motor from -9026 to -9000"
            in record.message
            for record in caplog.records
        ), [record.message for record in caplog.records]
    finally:
        _drop_pose(app, pose_id)


def test_position_update_clamps_as_well(app):
    """Re-recording an existing pose goes through PATCH and needs the same guard."""
    pose_id = _store_pose(app, "Clamp Test Update", [("turn_head_motor", 0)])
    try:
        with app.test_client() as client:
            response = client.patch(
                f"/pose/{pose_id}/motor-positions",
                json={
                    "motorPositions": [
                        {"motorName": "turn_head_motor", "position": 9026}
                    ]
                },
            )
        assert response.status_code == 200, response.get_data(as_text=True)
        assert _stored_positions(app, pose_id)["turn_head_motor"] == 9000
    finally:
        _drop_pose(app, pose_id)


def test_update_keeps_positions_inside_the_range(app):
    pose_id = _store_pose(app, "Clamp Test Update In Range", [("turn_head_motor", 0)])
    try:
        with app.test_client() as client:
            response = client.patch(
                f"/pose/{pose_id}/motor-positions",
                json={
                    "motorPositions": [
                        {"motorName": "turn_head_motor", "position": -9026}
                    ]
                },
            )
        assert response.status_code == 200, response.get_data(as_text=True)
        assert _stored_positions(app, pose_id)["turn_head_motor"] == -9000
    finally:
        _drop_pose(app, pose_id)


def test_every_stored_pose_of_the_seed_is_playable(app):
    """Boot must not leave a pose that Blockly refuses (raise instead of moving)."""
    with app.app_context():
        poses = db.session.query(Pose).all()
        violations = [
            (pose.name, motor_position.motor_name, motor_position.position)
            for pose in poses
            for motor_position in pose.motor_positions
            if not -9000 <= motor_position.position <= 9000
        ]
    assert not violations, violations
