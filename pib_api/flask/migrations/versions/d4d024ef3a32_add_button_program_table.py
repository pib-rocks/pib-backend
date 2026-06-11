"""add_button_programs_table

Revision ID: d4d024ef3a32
Revises: e125aa01171c
Create Date: 2026-03-17 13:53:24.716901

"""

from alembic import op
import sqlalchemy as sa

# revision identifiers, used by Alembic.
revision = "d4d024ef3a32"
down_revision = "e125aa01171c"
branch_labels = None
depends_on = None


def upgrade():
    op.create_table(
        "button_program",
        sa.Column("id", sa.Integer(), nullable=False),
        sa.Column("bricklet_id", sa.Integer(), nullable=False),
        sa.Column("program_id", sa.Integer(), nullable=True),
        sa.ForeignKeyConstraint(
            ["bricklet_id"],
            ["bricklet.id"],
        ),
        sa.ForeignKeyConstraint(["program_id"], ["program.id"], ondelete="SET NULL"),
        sa.PrimaryKeyConstraint("id"),
        sa.UniqueConstraint("bricklet_id"),
    )
    conn = op.get_bind()
    conn.execute(sa.text("""
        INSERT OR IGNORE INTO bricklet (type, uid, bricklet_number) VALUES 
        ('RGB LED Button Bricklet', null, 5),
        ('RGB LED Button Bricklet', null, 6),
        ('RGB LED Button Bricklet', null, 7);
            """))
    conn.execute(sa.text("""
        INSERT OR IGNORE INTO button_program (bricklet_id, program_id) SELECT id, NULL FROM bricklet WHERE type = 'RGB LED Button Bricklet';
            """))


def downgrade():
    op.drop_table("button_program")
    conn = op.get_bind()
    conn.execute(sa.text("""
        DELETE FROM bricklet WHERE type = 'RGB LED Button Bricklet';
            """))
