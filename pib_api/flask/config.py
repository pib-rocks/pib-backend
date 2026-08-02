import os
from os.path import dirname, abspath, expanduser

BASE_DIR = dirname(abspath(__file__))


class Config:
    SQLALCHEMY_DATABASE_URI = os.getenv(
        "SQLALCHEMY_DATABASE_URI", f"sqlite:///{os.path.join(BASE_DIR, 'pibdata.db')}"
    )
    # Wait up to 15s for SQLite locks instead of failing immediately under concurrency.
    SQLALCHEMY_ENGINE_OPTIONS = {"connect_args": {"timeout": 15.0}}
    PYTHON_CODE_DIR = os.getenv(
        "PYTHON_CODE_DIR", os.path.join(expanduser("~"), "cerebra_programs")
    )
