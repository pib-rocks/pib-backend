import os
from os.path import dirname, abspath, expanduser

BASE_DIR = dirname(dirname(abspath(__file__)))


class Config:
    SQLALCHEMY_DATABASE_URI = os.getenv(
        "SQLALCHEMY_DATABASE_URI", f"sqlite:///{os.path.join(BASE_DIR, 'pibdata.db')}"
    )
    PYTHON_CODE_DIR = os.getenv(
        "PYTHON_CODE_DIR", os.path.join(expanduser("~"), "cerebra_programs")
    )
