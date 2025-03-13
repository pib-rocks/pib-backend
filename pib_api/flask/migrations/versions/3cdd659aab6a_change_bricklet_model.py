"""change_bricklet_model

Revision ID: 3cdd659aab6a
Revises: c2de601304c2
Create Date: 2025-03-04 10:59:04.861783

"""

from alembic import op
import sqlalchemy as sa


# revision identifiers, used by Alembic.
revision = "3cdd659aab6a"
down_revision = "c2de601304c2"
branch_labels = None
depends_on = None


def upgrade():
    with op.batch_alter_table("bricklet", schema=None) as batch_op:
        batch_op.alter_column("uid", existing_type=sa.VARCHAR(length=30), nullable=True)


def downgrade():
    conn = op.get_bind()
    result = conn.execute(sa.text("SELECT id from bricklet WHERE uid IS NULL"))

    for row in result.fetchall():
        conn.execute(
            sa.text("UPDATE bricklet SET uid = :uid WHERE id = :id"),
            {"uid": f"EMPTY{row.id}", "id": row.id},
        )

    with op.batch_alter_table("bricklet", schema=None) as batch_op:
        batch_op.alter_column("uid", nullable=False)
