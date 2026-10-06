"""Live pytest E2E session.

``PIB_ROBOT_URL`` is the robot address. The API address is derived from it.
See ``tests/README.md`` and ``robot_address.py``.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

_E2E_DIR = Path(__file__).resolve().parent
if str(_E2E_DIR) not in sys.path:
    sys.path.insert(0, str(_E2E_DIR))

_MODULE_NAME = "pib_live_e2e_robot_address"
_MODULE_PATH = _E2E_DIR / "robot_address.py"


def _robot_address():
    loaded = sys.modules.get(_MODULE_NAME)
    if loaded is not None:
        return loaded
    spec = importlib.util.spec_from_file_location(_MODULE_NAME, _MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[_MODULE_NAME] = module
    spec.loader.exec_module(module)
    return module


_address = _robot_address()
RobotAddressError = _address.RobotAddressError
resolve = _address.resolve


def _item_path(item) -> Path:
    path = getattr(item, "path", None)
    if path is None:
        path = Path(str(item.fspath))
    return Path(path)


def _live_items(session):
    return [
        item for item in session.items if _item_path(item).resolve().parent == _E2E_DIR
    ]


def _announce(session, resolved) -> None:
    reporter = session.config.pluginmanager.getplugin("terminalreporter")
    message = resolved.summary()
    if reporter is not None:
        reporter.write_line(message)
    else:
        print(message)


@pytest.hookimpl(tryfirst=True)
def pytest_runtestloop(session):
    """Stop an unconfigured live run before any test contacts localhost.

    ``--collect-only`` still enters this hook; it must keep collecting.
    A mixed session that also contains unit tests keeps running; each live
    test then fails in setup with the same message.
    """
    if session.config.option.collectonly:
        return None
    items = _live_items(session)
    if not items:
        return None
    try:
        resolved = resolve()
    except RobotAddressError as error:
        if len(items) == len(session.items):
            message = str(error)
            reporter = session.config.pluginmanager.getplugin("terminalreporter")
            if reporter is not None:
                reporter.write_line(message)
            pytest.exit(message, returncode=1)
        return None
    _announce(session, resolved)
    return None


def pytest_runtest_setup(item):
    if _item_path(item).resolve().parent != _E2E_DIR:
        return
    try:
        resolve()
    except RobotAddressError as error:
        pytest.fail(str(error), pytrace=False)
