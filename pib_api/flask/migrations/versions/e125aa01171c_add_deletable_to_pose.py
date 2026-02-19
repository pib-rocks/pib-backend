"""add deletable to pose

Revision ID: e125aa01171c
Revises: ba9e8ed1e18c
Create Date: 2025-09-11 13:54:41.677586

"""

from alembic import op
import sqlalchemy as sa

# revision identifiers, used by Alembic.
revision = "e125aa01171c"
down_revision = "ba9e8ed1e18c"
branch_labels = None
depends_on = None


def upgrade():
    # add deletable column
    with op.batch_alter_table("pose", schema=None) as batch_op:
        batch_op.add_column(
            sa.Column(
                "deletable", sa.Boolean(), nullable=False, server_default=sa.true()
            )
        )


def downgrade():
    # remove deletable column but keep default poses
    with op.batch_alter_table("pose", schema=None) as batch_op:
        batch_op.drop_column("deletable")
