"""add bricklet type

Revision ID: ddd0daa8e6f5
Revises: 3cdd659aab6a
Create Date: 2025-05-06 09:53:52.143007

"""

from alembic import op
import sqlalchemy as sa


# revision identifiers, used by Alembic.
revision = "ddd0daa8e6f5"
down_revision = "3cdd659aab6a"
branch_labels = None
depends_on = None


def upgrade():
    with op.batch_alter_table("bricklet", schema=None) as batch_op:
        batch_op.add_column(
            sa.Column(
                "type",
                sa.Enum(
                    "Solid State Relay Bricklet", "Servo Bricklet", name="bricklet_type"
                ),
                nullable=False,
                server_default="Servo Bricklet",
            )
        )


def downgrade():
    with op.batch_alter_table("bricklet", schema=None) as batch_op:
        op.execute("DELETE FROM bricklet WHERE type = 'Solid State Relay Bricklet'")
        batch_op.drop_column("type")
