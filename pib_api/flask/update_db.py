from app.app import db, app
from default_pose_constants import (
    STARTUP_POSITIONS,
    CALIBRATION_POSITIONS,
    STARTUP_POSE_NAME,
    CALIBRATION_POSE_NAME,
)
from model.motor_position_model import MotorPosition
from model.pose_model import Pose
from model.util import generate_uuid


def update_db():
    """Ensure required default poses exist with correct motor positions.

    Creates poses if they don't exist. For Calibration pose, always updates
    motor positions to defaults. For Startup/Resting pose, only creates
    initial positions (preserves user modifications).
    """
    default_poses = {
        STARTUP_POSE_NAME: STARTUP_POSITIONS,
        CALIBRATION_POSE_NAME: CALIBRATION_POSITIONS,
    }
    with app.app_context():
        for pose_name, motor_positions in default_poses.items():
            pose = db.session.query(Pose).filter_by(name=pose_name).first()

            if not pose:
                # Create new pose with motor positions
                pose = Pose(pose_id=generate_uuid(), name=pose_name, deletable=False)
                db.session.add(pose)
                db.session.flush()

                db.session.add_all(
                    MotorPosition(pose_id=pose.pose_id, motor_name=motor, position=pos)
                    for motor, pos in motor_positions.items()
                )
            elif pose_name == CALIBRATION_POSE_NAME:
                # Always reset Calibration pose to defaults
                db.session.query(MotorPosition).filter_by(pose_id=pose.pose_id).delete()
                db.session.add_all(
                    MotorPosition(pose_id=pose.pose_id, motor_name=motor, position=pos)
                    for motor, pos in motor_positions.items()
                )
            # else: Startup/Resting pose exists - preserve user modifications

        db.session.commit()
        print("Database updated with default poses.")


if __name__ == "__main__":
    update_db()
