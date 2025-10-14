"""add deletable to pose

Revision ID: e125aa01171c
Revises: ddd0daa8e6f5
Create Date: 2025-09-11 13:54:41.677586

"""

from model.util import generate_uuid
from alembic import op
import sqlalchemy as sa


# revision identifiers, used by Alembic.
revision = "e125aa01171c"
down_revision = "ddd0daa8e6f5"
branch_labels = None
depends_on = None

all_motors = [
    "turn_head_motor",
    "tilt_forward_motor",
    "upper_arm_left_rotation",
    "elbow_left",
    "lower_arm_left_rotation",
    "shoulder_vertical_left",
    "shoulder_horizontal_left",
    "upper_arm_right_rotation",
    "elbow_right",
    "lower_arm_right_rotation",
    "shoulder_vertical_right",
    "shoulder_horizontal_right",
    "thumb_right_opposition",
    "thumb_right_stretch",
    "index_right_stretch",
    "middle_right_stretch",
    "ring_right_stretch",
    "pinky_right_stretch",
    "thumb_left_opposition",
    "thumb_left_stretch",
    "index_left_stretch",
    "middle_left_stretch",
    "ring_left_stretch",
    "pinky_left_stretch",
    "wrist_left",
    "wrist_right",
]

overrides_startup = {
    "elbow_left": 5500,
    "elbow_right": 5500,
    "thumb_right_opposition": -9000,
    "thumb_right_stretch": -9000,
    "index_right_stretch": -9000,
    "middle_right_stretch": -9000,
    "ring_right_stretch": -9000,
    "pinky_right_stretch": -9000,
    "thumb_left_opposition": -9000,
    "thumb_left_stretch": -9000,
    "index_left_stretch": -9000,
    "middle_left_stretch": -9000,
    "ring_left_stretch": -9000,
    "pinky_left_stretch": -9000,
    "shoulder_vertical_left": -9000,
    "shoulder_horizontal_left": -9000,
    "shoulder_vertical_right": -9000,
    "shoulder_horizontal_right": -9000,
}
overrides_calibration = {
    "thumb_right_opposition": -9000,
    "thumb_right_stretch": -9000,
    "index_right_stretch": -9000,
    "middle_right_stretch": -9000,
    "ring_right_stretch": -9000,
    "pinky_right_stretch": -9000,
    "thumb_left_opposition": -9000,
    "thumb_left_stretch": -9000,
    "index_left_stretch": -9000,
    "middle_left_stretch": -9000,
    "ring_left_stretch": -9000,
    "pinky_left_stretch": -9000,
}


def upgrade():
    # add deletable column
    with op.batch_alter_table("pose", schema=None) as batch_op:
        batch_op.add_column(
            sa.Column(
                "deletable", sa.Boolean(), nullable=False, server_default=sa.true()
            )
        )

    # add default poses when they don't exist
    op.execute(
        f"INSERT OR IGNORE INTO pose (pose_id, name, deletable) VALUES ('{generate_uuid()}', 'Startup/Resting', 0)"
    )
    op.execute(
        f"INSERT OR IGNORE INTO pose (pose_id, name, deletable) VALUES ('{generate_uuid()}', 'Calibration', 0)"
    )

    # make sure default poses are not deletable
    op.execute(
        "UPDATE pose SET deletable=0 WHERE name IN ('Startup/Resting', 'Calibration')"
    )

    # add startup positions
    startup_values = " UNION ALL ".join(
        f"SELECT '{motor}', {overrides_startup.get(motor, 0)}, id FROM pose WHERE name='Startup/Resting'"
        for motor in all_motors
    )
    op.execute(
        f"INSERT OR IGNORE INTO motor_position (motor_name, position, pose_id) {startup_values}"
    )

    # add calibration positions
    calibration_values = " UNION ALL ".join(
        f"SELECT '{motor}', {overrides_calibration.get(motor, 0)}, id FROM pose WHERE name='Calibration'"
        for motor in all_motors
    )
    op.execute(
        f"INSERT OR IGNORE INTO motor_position (motor_name, position, pose_id) {calibration_values}"
    )


def downgrade():
    # remove deletable column but keep default poses
    with op.batch_alter_table("pose", schema=None) as batch_op:
        batch_op.drop_column("deletable")
