"""Live pytest E2E session.

``PIB_ROBOT_URL`` is the robot address. The API address is derived from it.
See ``tests/README.md`` and ``robot_address.py``.
"""

from __future__ import annotations

import contextlib
import importlib.util
import sys
from pathlib import Path

import pytest
import requests

_E2E_DIR = Path(__file__).resolve().parent
if str(_E2E_DIR) not in sys.path:
    sys.path.insert(0, str(_E2E_DIR))

_MODULE_NAME = "pib_live_e2e_robot_address"
_MODULE_PATH = _E2E_DIR / "robot_address.py"

_SMART_CONNECT_NAME = "pib_live_e2e_smart_connect_token"
_SMART_CONNECT_PATH = _E2E_DIR / "smart_connect_token.py"


def _load(name: str, path: Path):
    loaded = sys.modules.get(name)
    if loaded is not None:
        return loaded
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def _robot_address():
    return _load(_MODULE_NAME, _MODULE_PATH)


def _smart_connect():
    return _load(_SMART_CONNECT_NAME, _SMART_CONNECT_PATH)


_address = _robot_address()
RobotAddressError = _address.RobotAddressError
resolve = _address.resolve
api_url = _address.api_url

_connect = _smart_connect()
ensure_smart_connect_token = _connect.ensure_smart_connect_token
delete_cloud_token = _connect.delete_cloud_token


@pytest.fixture(scope="session", autouse=True)
def smart_connect_token_state():
    """Give the suite a SmartConnect token without ever replacing a real one.

    The robot's key store holds the operator's real pib.Cloud token. Writing a
    test placeholder over it leaves the robot answering ``401`` on every cloud
    turn until someone re-enters the token, so the suite only writes when the
    store is empty - and removes that placeholder again at the end of the run.
    See ``smart_connect_token.py``.
    """
    base = None
    created = False
    try:
        base = api_url()
        created = ensure_smart_connect_token(base)
    except (RobotAddressError, requests.RequestException):
        # Unreachable robot or unset address: the live tests report that
        # themselves. Never fail the session setup for it.
        created = False
    yield created
    if created:
        with contextlib.suppress(requests.RequestException):
            delete_cloud_token(base)


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
