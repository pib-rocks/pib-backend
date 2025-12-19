from app.app import db, app
from default_pose_constants import STARTUP_POSITIONS, CALIBRATION_POSITIONS
from model.motor_position_model import MotorPosition
from model.pose_model import Pose
from model.util import generate_uuid


def update_db():
    """Ensure required default poses exist and have correct motor positions."""
    default_poses = {
        "Startup/Resting": STARTUP_POSITIONS,
        "Calibration": CALIBRATION_POSITIONS,
    }
    with app.app_context():
        for pose_name, motor_positions in default_poses.items():

            pose = db.session.query(Pose).filter_by(name=pose_name).first()
            if not pose:
                pose = Pose(pose_id=generate_uuid(), name=pose_name, deletable=False)
                db.session.add(pose)

            db.session.query(MotorPosition).filter_by(pose_id=pose.pose_id).delete()

            db.session.add_all(
                MotorPosition(pose_id=pose.pose_id, motor_name=motor, position=pos)
                for motor, pos in motor_positions.items()
            )

        db.session.commit()
        print("Database updated with default poses.")


if __name__ == "__main__":
    update_db()
