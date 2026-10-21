"""add the chat message image table

Revision ID: d7a1c4e29b36
Revises: c4e9a1b73f28
Create Date: 2026-10-21 14:05:00.000000

An image a tool captured during a turn is stored beside its chat message and the
message keeps only a reference (PC-119). ``GET /chat/<id>/messages`` answers with
every message of a chat and no paging, so inline base64 would ride along on every
load of the history and grow the database by 4/3 of each picture, forever.

No backfill: nothing captured a picture before this revision, and an existing
message simply has no images.
"""

import sqlalchemy as sa
from alembic import op

revision = "d7a1c4e29b36"
down_revision = "c4e9a1b73f28"
branch_labels = None
depends_on = None


def _table_exists(conn) -> bool:
    rows = conn.execute(sa.text("PRAGMA table_info(chatMessageImage)")).fetchall()
    return bool(rows)


def upgrade():
    conn = op.get_bind()
    if _table_exists(conn):
        return
    op.create_table(
        "chatMessageImage",
        sa.Column("id", sa.String(length=255), nullable=False),
        sa.Column("sequence", sa.Integer(), nullable=False),
        sa.Column("message_id", sa.String(length=255), nullable=False),
        sa.Column("mime_type", sa.String(length=64), nullable=False),
        sa.Column("byte_size", sa.Integer(), nullable=False),
        sa.Column("width", sa.Integer(), nullable=True),
        sa.Column("height", sa.Integer(), nullable=True),
        sa.Column("file_name", sa.String(length=255), nullable=False),
        sa.Column("created_at", sa.DateTime(), nullable=False),
        sa.PrimaryKeyConstraint("id"),
    )
    op.create_index(
        "ix_chat_message_image_message_id",
        "chatMessageImage",
        ["message_id"],
        unique=False,
    )


def downgrade():
    conn = op.get_bind()
    if not _table_exists(conn):
        return
    op.drop_index("ix_chat_message_image_message_id", table_name="chatMessageImage")
    op.drop_table("chatMessageImage")
