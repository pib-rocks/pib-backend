"""add pose- and motor-position-table

Revision ID: 7160a4f6951f
Revises: 7a4053561c44
Create Date: 2024-06-03 13:54:28.623744

"""

from alembic import op
import sqlalchemy as sa


# revision identifiers, used by Alembic.
revision = "7160a4f6951f"
down_revision = "7a4053561c44"
branch_labels = None
depends_on = None


def upgrade():
    # ### commands auto generated by Alembic - please adjust! ###
    op.create_table(
        "pose",
        sa.Column("id", sa.Integer(), nullable=False),
        sa.Column("pose_id", sa.String(length=255), nullable=False),
        sa.Column("name", sa.String(length=255), nullable=False),
        sa.PrimaryKeyConstraint("id"),
        sa.UniqueConstraint("name"),
        sa.UniqueConstraint("pose_id"),
    )
    op.create_table(
        "motor_position",
        sa.Column("id", sa.Integer(), nullable=False),
        sa.Column("position", sa.Integer(), nullable=False),
        sa.Column("motor_name", sa.Integer(), nullable=False),
        sa.Column("pose_id", sa.Integer(), nullable=False),
        sa.ForeignKeyConstraint(
            ["motor_name"],
            ["motor.name"],
        ),
        sa.ForeignKeyConstraint(
            ["pose_id"],
            ["pose.id"],
        ),
        sa.PrimaryKeyConstraint("id"),
    )
    # ### end Alembic commands ###


def downgrade():
    # ### commands auto generated by Alembic - please adjust! ###
    op.drop_table("motor_position")
    op.drop_table("pose")
    # ### end Alembic commands ###