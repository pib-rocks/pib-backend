"""replace Bricklet pin mapping with generic controllers

Revision ID: a1638c0de001
Revises: f3a1b2c4d5e6
Create Date: 2026-09-17 14:30:00
"""

from alembic import op
import sqlalchemy as sa

revision = "a1638c0de001"
down_revision = "f3a1b2c4d5e6"
branch_labels = None
depends_on = None

FK_NAMING_CONVENTION = {
    "fk": "fk_%(table_name)s_%(column_0_name)s_%(referred_table_name)s"
}


def upgrade():
    op.create_table(
        "controller",
        sa.Column("id", sa.Integer(), nullable=False),
        sa.Column("kind", sa.String(length=50), nullable=False),
        sa.Column("device_type", sa.String(length=50), nullable=True),
        sa.Column("address", sa.String(length=255), nullable=True),
        sa.Column("number", sa.Integer(), nullable=False),
        sa.Column("supply_voltage", sa.Float(), nullable=True),
        sa.Column(
            "created_at",
            sa.DateTime(),
            nullable=False,
            server_default=sa.func.current_timestamp(),
        ),
        sa.Column(
            "updated_at",
            sa.DateTime(),
            nullable=False,
            server_default=sa.func.current_timestamp(),
        ),
        sa.PrimaryKeyConstraint("id"),
        sa.UniqueConstraint("number"),
    )

    connection = op.get_bind()
    connection.execute(sa.text("""
            INSERT INTO controller
                (id, kind, device_type, address, number, supply_voltage,
                 created_at, updated_at)
            SELECT id, 'tinkerforge_bricklet', type, uid, bricklet_number,
                   NULL, CURRENT_TIMESTAMP, CURRENT_TIMESTAMP
            FROM bricklet
            """))

    with op.batch_alter_table("motor") as batch_op:
        batch_op.add_column(sa.Column("controller_id", sa.Integer(), nullable=True))
        batch_op.add_column(sa.Column("channel", sa.Integer(), nullable=True))
        batch_op.add_column(sa.Column("current_limit", sa.Float(), nullable=True))
        batch_op.add_column(sa.Column("torque_limit", sa.Float(), nullable=True))
        batch_op.create_foreign_key(
            "fk_motor_controller_id_controller",
            "controller",
            ["controller_id"],
            ["id"],
        )

    # Historical data is expected to have one mapping per motor. If malformed
    # data has several, retain the first row by primary key deterministically.
    connection.execute(sa.text("""
            UPDATE motor
            SET controller_id = (
                    SELECT bp.bricklet_id
                    FROM "brickletPin" AS bp
                    WHERE bp.motor_id = motor.id
                    ORDER BY bp.id
                    LIMIT 1
                ),
                channel = (
                    SELECT bp.pin
                    FROM "brickletPin" AS bp
                    WHERE bp.motor_id = motor.id
                    ORDER BY bp.id
                    LIMIT 1
                )
            """))

    with op.batch_alter_table(
        "button_program",
        recreate="always",
        naming_convention=FK_NAMING_CONVENTION,
    ) as batch_op:
        batch_op.drop_constraint(
            "fk_button_program_bricklet_id_bricklet", type_="foreignkey"
        )
        batch_op.alter_column(
            "bricklet_id",
            new_column_name="controller_id",
            existing_type=sa.Integer(),
            existing_nullable=False,
        )
        batch_op.create_foreign_key(
            "fk_button_program_controller_id_controller",
            "controller",
            ["controller_id"],
            ["id"],
        )

    op.drop_table("brickletPin")
    op.drop_table("bricklet")


def downgrade():
    op.create_table(
        "bricklet",
        sa.Column("id", sa.Integer(), nullable=False),
        sa.Column("uid", sa.String(length=30), nullable=True),
        sa.Column("bricklet_number", sa.Integer(), nullable=False),
        sa.Column("type", sa.String(length=32), nullable=False),
        sa.PrimaryKeyConstraint("id"),
        sa.UniqueConstraint("bricklet_number"),
        sa.UniqueConstraint("uid"),
    )
    connection = op.get_bind()
    connection.execute(sa.text("""
            INSERT INTO bricklet (id, uid, bricklet_number, type)
            SELECT id, address, number,
                   COALESCE(
                       device_type,
                       CASE
                           WHEN number = 4 THEN 'Solid State Relay Bricklet'
                           WHEN number IN (5, 6, 7)
                               THEN 'RGB LED Button Bricklet'
                           ELSE 'Servo Bricklet'
                       END
                   )
            FROM controller
            WHERE kind = 'tinkerforge_bricklet'
            """))

    with op.batch_alter_table(
        "button_program",
        recreate="always",
        naming_convention=FK_NAMING_CONVENTION,
    ) as batch_op:
        batch_op.drop_constraint(
            "fk_button_program_controller_id_controller", type_="foreignkey"
        )
        batch_op.alter_column(
            "controller_id",
            new_column_name="bricklet_id",
            existing_type=sa.Integer(),
            existing_nullable=False,
        )
        batch_op.create_foreign_key(
            "fk_button_program_bricklet_id_bricklet",
            "bricklet",
            ["bricklet_id"],
            ["id"],
        )

    op.create_table(
        "brickletPin",
        sa.Column("id", sa.Integer(), nullable=False),
        sa.Column("motor_id", sa.Integer(), nullable=False),
        sa.Column("bricklet_id", sa.Integer(), nullable=False),
        sa.Column("pin", sa.Integer(), nullable=False),
        sa.Column("invert", sa.Boolean(), nullable=False),
        sa.ForeignKeyConstraint(["bricklet_id"], ["bricklet.id"]),
        sa.ForeignKeyConstraint(["motor_id"], ["motor.id"]),
        sa.PrimaryKeyConstraint("id"),
    )
    connection.execute(sa.text("""
            INSERT INTO "brickletPin" (motor_id, bricklet_id, pin, invert)
            SELECT id, controller_id, channel, invert
            FROM motor
            WHERE controller_id IS NOT NULL AND channel IS NOT NULL
            """))

    with op.batch_alter_table("motor") as batch_op:
        batch_op.drop_constraint(
            "fk_motor_controller_id_controller", type_="foreignkey"
        )
        batch_op.drop_column("torque_limit")
        batch_op.drop_column("current_limit")
        batch_op.drop_column("channel")
        batch_op.drop_column("controller_id")
    op.drop_table("controller")
