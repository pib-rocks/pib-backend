"""A pose that a Blockly program still points at may not be deleted.

A stored program keeps the pose UUID it was built with. Re-recording a pose means deleting
it and creating a new one, which allocates a new UUID: the program then references a pose
that no longer exists. Measured on .168, running such a program ended in
``BackendError: GET /pose/by-name/<uuid> -> HTTP 404`` with a long traceback and the robot
did not move - which reads as "my new pose does not work".

Refusing the delete keeps the reference graph consistent; the pose can still be updated in
place (PATCH), which keeps its UUID and therefore the program.
"""

from app.app import db
from model.pose_model import Pose
from model.program_model import Program
from service import pose_service


def _create_pose(app, name):
    with app.app_context():
        pose = pose_service.create_pose(
            {
                "name": name,
                "motor_positions": [{"motor_name": "turn_head_motor", "position": 0}],
            }
        )
        db.session.commit()
        return pose.pose_id


def _create_program(app, name, pose_id):
    with app.app_context():
        program = Program(
            name=name,
            code_visual=(
                '{"blocks":{"languageVersion":0,"blocks":[{"type":"move_to_pose",'
                f'"fields":{{"POSE":"{pose_id}"}}}}]}}}}'
            ),
        )
        db.session.add(program)
        db.session.commit()
        return program.program_number


def _cleanup(app, pose_id=None, program_number=None):
    with app.app_context():
        if program_number:
            db.session.query(Program).filter_by(program_number=program_number).delete()
        if pose_id:
            db.session.query(Pose).filter_by(pose_id=pose_id).delete()
        db.session.commit()


def test_deleting_a_pose_used_by_a_program_is_refused(app):
    pose_id = _create_pose(app, "Referenced Pose")
    program_number = _create_program(app, "Program using the pose", pose_id)
    try:
        with app.test_client() as client:
            response = client.delete(f"/pose/{pose_id}")

        assert response.status_code == 409, response.get_data(as_text=True)
        error = response.get_json()["error"]
        assert "Referenced Pose" in error
        assert "Program using the pose" in error

        with app.app_context():
            assert db.session.query(Pose).filter_by(pose_id=pose_id).one() is not None
    finally:
        _cleanup(app, pose_id=pose_id, program_number=program_number)


def test_deleting_an_unreferenced_pose_still_works(app):
    pose_id = _create_pose(app, "Unreferenced Pose")

    with app.test_client() as client:
        response = client.delete(f"/pose/{pose_id}")

    assert response.status_code == 204, response.get_data(as_text=True)
    with app.app_context():
        assert db.session.query(Pose).filter_by(pose_id=pose_id).first() is None


def test_another_pose_in_the_program_does_not_block(app):
    """Only the pose the program actually names counts, not any program at all."""
    pose_id = _create_pose(app, "Pose without program")
    other_pose_id = _create_pose(app, "Pose inside the program")
    program_number = _create_program(app, "Program of another pose", other_pose_id)
    try:
        with app.test_client() as client:
            response = client.delete(f"/pose/{pose_id}")

        assert response.status_code == 204, response.get_data(as_text=True)
    finally:
        _cleanup(app, pose_id=other_pose_id, program_number=program_number)


def test_service_names_the_refusal(app):
    pose_id = _create_pose(app, "Referenced Pose Service")
    program_number = _create_program(app, "Program in service test", pose_id)
    try:
        with app.app_context():
            try:
                pose_service.delete_pose(pose_id)
            except pose_service.PoseRefusedError as error:
                assert "Program in service test" in str(error)
            else:  # pragma: no cover - the assertion below reports it
                raise AssertionError("delete_pose did not refuse the referenced pose")
    finally:
        _cleanup(app, pose_id=pose_id, program_number=program_number)
