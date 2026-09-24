from __future__ import annotations

import importlib.util
from pathlib import Path

import sqlalchemy as sa
from alembic.migration import MigrationContext
from alembic.operations import Operations

REPO_ROOT = Path(__file__).resolve().parents[2]
MIGRATION_PATH = (
    REPO_ROOT
    / "pib_api"
    / "flask"
    / "migrations"
    / "versions"
    / "a1638c0de001_generic_controller.py"
)


def _load_migration():
    spec = importlib.util.spec_from_file_location(
        "generic_controller_migration", MIGRATION_PATH
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_upgrade_preserves_every_motor_controller_channel_mapping(tmp_path):
    engine = sa.create_engine(f"sqlite:///{tmp_path / 'old-schema.db'}")
    metadata = sa.MetaData()
    bricklet = sa.Table(
        "bricklet",
        metadata,
        sa.Column("id", sa.Integer, primary_key=True),
        sa.Column("uid", sa.String(30), unique=True),
        sa.Column("bricklet_number", sa.Integer, nullable=False, unique=True),
        sa.Column("type", sa.String(32), nullable=False),
    )
    motor = sa.Table(
        "motor",
        metadata,
        sa.Column("id", sa.Integer, primary_key=True),
        sa.Column("name", sa.String(255), nullable=False, unique=True),
        sa.Column("invert", sa.Boolean, nullable=False),
    )
    bricklet_pin = sa.Table(
        "brickletPin",
        metadata,
        sa.Column("id", sa.Integer, primary_key=True),
        sa.Column("motor_id", sa.Integer, sa.ForeignKey("motor.id"), nullable=False),
        sa.Column(
            "bricklet_id",
            sa.Integer,
            sa.ForeignKey("bricklet.id"),
            nullable=False,
        ),
        sa.Column("pin", sa.Integer, nullable=False),
        sa.Column("invert", sa.Boolean, nullable=False),
    )
    program = sa.Table(
        "program",
        metadata,
        sa.Column("id", sa.Integer, primary_key=True),
    )
    button_program = sa.Table(
        "button_program",
        metadata,
        sa.Column("id", sa.Integer, primary_key=True),
        sa.Column(
            "bricklet_id",
            sa.Integer,
            sa.ForeignKey("bricklet.id"),
            nullable=False,
            unique=True,
        ),
        sa.Column("program_id", sa.Integer, sa.ForeignKey("program.id")),
    )
    metadata.create_all(engine)

    with engine.begin() as connection:
        connection.execute(
            bricklet.insert(),
            [
                {
                    "id": 7,
                    "uid": "ABC123",
                    "bricklet_number": 1,
                    "type": "Servo Bricklet",
                },
                {
                    "id": 9,
                    "uid": "XYZ789",
                    "bricklet_number": 2,
                    "type": "Solid State Relay Bricklet",
                },
            ],
        )
        connection.execute(
            motor.insert(),
            [
                {"id": 11, "name": "left", "invert": False},
                {"id": 12, "name": "right", "invert": True},
            ],
        )
        connection.execute(
            bricklet_pin.insert(),
            [
                {
                    "id": 20,
                    "motor_id": 11,
                    "bricklet_id": 7,
                    "pin": 3,
                    "invert": False,
                },
                {
                    "id": 21,
                    "motor_id": 12,
                    "bricklet_id": 9,
                    "pin": 8,
                    "invert": False,
                },
            ],
        )
        connection.execute(button_program.insert(), [{"id": 1, "bricklet_id": 7}])
        before = connection.execute(sa.text("""
                SELECT motor.id, "brickletPin".bricklet_id, "brickletPin".pin
                FROM motor
                JOIN "brickletPin" ON "brickletPin".motor_id = motor.id
                ORDER BY motor.id
                """)).all()
        device_types_before = connection.execute(sa.text("""
                SELECT id, type
                FROM bricklet
                ORDER BY id
                """)).all()

        context = MigrationContext.configure(connection)
        with Operations.context(context):
            _load_migration().upgrade()

        after = connection.execute(sa.text("""
                SELECT id, controller_id, channel
                FROM motor
                ORDER BY id
                """)).all()
        controllers = connection.execute(sa.text("""
                SELECT id, kind, device_type, address, number, supply_voltage
                FROM controller
                ORDER BY id
                """)).all()
        device_types_after = connection.execute(sa.text("""
                SELECT id, device_type
                FROM controller
                ORDER BY id
                """)).all()
        button_mapping = connection.execute(
            sa.text("SELECT controller_id FROM button_program")
        ).scalar_one()
        tables = set(sa.inspect(connection).get_table_names())

    assert after == before
    assert all(
        row.controller_id is not None and row.channel is not None for row in after
    )
    assert controllers == [
        (7, "tinkerforge_bricklet", "Servo Bricklet", "ABC123", 1, None),
        (
            9,
            "tinkerforge_bricklet",
            "Solid State Relay Bricklet",
            "XYZ789",
            2,
            None,
        ),
    ]
    assert device_types_after == device_types_before
    assert button_mapping == 7
    assert "brickletPin" not in tables
    assert "bricklet" not in tables
