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

# TODO: add real startup positions and add calibration positions
motor_positions_startup = {
    "turn_head_motor": -4000,
    "tilt_forward_motor": 0,
    "upper_arm_left_rotation": -4000,
    "elbow_left": 0,
    "lower_arm_left_rotation": 0,
    "shoulder_vertical_left": 0,
    "shoulder_horizontal_left": 0,
    "upper_arm_right_rotation": -4000,
    "elbow_right": 0,
    "lower_arm_right_rotation": 0,
    "shoulder_vertical_right": 0,
    "shoulder_horizontal_right": 0,
    "thumb_right_opposition": 0,
    "thumb_right_stretch": 0,
    "index_right_stretch": 0,
    "middle_right_stretch": 0,
    "ring_right_stretch": 0,
    "pinky_right_stretch": 0,
    "thumb_left_opposition": 0,
    "thumb_left_stretch": 0,
    "index_left_stretch": 0,
    "middle_left_stretch": 0,
    "ring_left_stretch": 0,
    "pinky_left_stretch": 0,
    "wrist_left": 0,
    "wrist_right": 0,
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
        f"SELECT '{motor}', {pos}, id FROM pose WHERE name='Startup/Resting'"
        for motor, pos in motor_positions_startup.items()
    )
    op.execute(
        f"INSERT OR IGNORE INTO motor_position (motor_name, position, pose_id) {startup_values}"
    )

    # add calibration positions
    calibration_values = " UNION ALL ".join(
        f"SELECT '{motor}', 0, id FROM pose WHERE name='Calibration'"
        for motor in motor_positions_startup.keys()
    )
    op.execute(
        f"INSERT OR IGNORE INTO motor_position (motor_name, position, pose_id) {calibration_values}"
    )


def downgrade():
    # remove deletable column but keep default poses
    with op.batch_alter_table("pose", schema=None) as batch_op:
        batch_op.drop_column("deletable")
