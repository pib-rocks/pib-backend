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
    / "e8f1a2b3c4d5_motor_upper_endpoint.py"
)


def _load_migration():
    spec = importlib.util.spec_from_file_location(
        "upper_endpoint_migration", MIGRATION_PATH
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_upgrade_maps_invert_to_upper_endpoint(tmp_path):
    engine = sa.create_engine(f"sqlite:///{tmp_path / 'motor.db'}")
    metadata = sa.MetaData()
    motor = sa.Table(
        "motor",
        metadata,
        sa.Column("id", sa.Integer, primary_key=True),
        sa.Column("name", sa.String(255), nullable=False),
        sa.Column("invert", sa.Boolean, nullable=False),
    )
    metadata.create_all(engine)

    with engine.begin() as connection:
        connection.execute(
            motor.insert(),
            [
                {"id": 1, "name": "plain", "invert": False},
                {"id": 2, "name": "flipped", "invert": True},
            ],
        )

        context = MigrationContext.configure(connection)
        with Operations.context(context):
            _load_migration().upgrade()

        rows = {
            row.name: row.upper_endpoint
            for row in connection.execute(
                sa.text("SELECT name, upper_endpoint FROM motor ORDER BY id")
            )
        }

    assert rows == {"plain": "max", "flipped": "min"}
    assert "invert" not in sa.inspect(engine).get_columns("motor")


def test_downgrade_restores_invert_from_upper_endpoint(tmp_path):
    engine = sa.create_engine(f"sqlite:///{tmp_path / 'motor.db'}")
    metadata = sa.MetaData()
    motor = sa.Table(
        "motor",
        metadata,
        sa.Column("id", sa.Integer, primary_key=True),
        sa.Column("name", sa.String(255), nullable=False),
        sa.Column("invert", sa.Boolean, nullable=False),
    )
    metadata.create_all(engine)

    migration = _load_migration()
    with engine.begin() as connection:
        connection.execute(
            motor.insert(),
            [
                {"id": 1, "name": "plain", "invert": False},
                {"id": 2, "name": "flipped", "invert": True},
            ],
        )
        context = MigrationContext.configure(connection)
        with Operations.context(context):
            migration.upgrade()
            migration.downgrade()

        rows = {
            row.name: row.invert
            for row in connection.execute(
                sa.text("SELECT name, invert FROM motor ORDER BY id")
            )
        }

    assert rows == {"plain": 0, "flipped": 1}
