"""pin the live model and the per-personality idle timeout

Revision ID: f1a9c3e74b20
Revises: a9c3e7b15d40
Create Date: 2026-09-30 06:00:00.000000

The Gemini identifier was read from this account's /v1beta/models list on
2026-09-30. gpt-realtime is not written here: no OpenAI key was available
to read /v1/models.

"""

import sqlalchemy as sa
from alembic import op

from pib_hermes_config.live_session import (
    DEFAULT_LIVE_IDLE_TIMEOUT_SECONDS,
    GEMINI_LIVE_MODEL,
    GEMINI_LIVE_MODEL_CHECKED_ON,
    VOICE_MODE_LIVE,
)
from provider_registry import gemini_live_chat_api_names

revision = "f1a9c3e74b20"
down_revision = "a9c3e7b15d40"
branch_labels = None
depends_on = None


def upgrade():
    with op.batch_alter_table("provider", schema=None) as batch_op:
        batch_op.add_column(
            sa.Column("live_model", sa.String(length=255), nullable=True)
        )
        batch_op.add_column(
            sa.Column("live_model_checked_on", sa.Date(), nullable=True)
        )

    conn = op.get_bind()
    # The catalogue names which Gemini chat rows carry the live model.
    for api_name in gemini_live_chat_api_names():
        conn.execute(
            sa.text("""
                UPDATE provider
                SET live_model = :live_model,
                    live_model_checked_on = :checked_on
                WHERE api_name = :api_name
                """),
            {
                "live_model": GEMINI_LIVE_MODEL,
                "checked_on": GEMINI_LIVE_MODEL_CHECKED_ON,
                "api_name": api_name,
            },
        )

    with op.batch_alter_table("personality", schema=None) as batch_op:
        batch_op.add_column(
            sa.Column(
                "voice_mode",
                sa.String(length=255),
                nullable=True,
                server_default=VOICE_MODE_LIVE,
            )
        )
        batch_op.add_column(
            sa.Column(
                "live_idle_timeout",
                sa.Integer(),
                nullable=True,
                server_default=str(DEFAULT_LIVE_IDLE_TIMEOUT_SECONDS),
            )
        )
    conn.execute(
        sa.text("""
            UPDATE personality
            SET voice_mode = :voice_mode
            WHERE voice_mode IS NULL
            """),
        {"voice_mode": VOICE_MODE_LIVE},
    )
    conn.execute(
        sa.text("""
            UPDATE personality
            SET live_idle_timeout = :timeout
            WHERE live_idle_timeout IS NULL
            """),
        {"timeout": DEFAULT_LIVE_IDLE_TIMEOUT_SECONDS},
    )
    with op.batch_alter_table("personality", schema=None) as batch_op:
        batch_op.alter_column(
            "voice_mode",
            existing_type=sa.String(length=255),
            nullable=False,
            server_default=VOICE_MODE_LIVE,
        )
        batch_op.alter_column(
            "live_idle_timeout",
            existing_type=sa.Integer(),
            nullable=False,
            server_default=str(DEFAULT_LIVE_IDLE_TIMEOUT_SECONDS),
        )


def downgrade():
    with op.batch_alter_table("personality", schema=None) as batch_op:
        batch_op.drop_column("live_idle_timeout")
        batch_op.drop_column("voice_mode")
    with op.batch_alter_table("provider", schema=None) as batch_op:
        batch_op.drop_column("live_model_checked_on")
        batch_op.drop_column("live_model")
