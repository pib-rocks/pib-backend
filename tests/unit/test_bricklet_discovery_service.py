"""Unit tests for the connected-Bricklet enumeration (PR-1854).

The connection is a stub. Nothing here talks to brickd or to a robot.
"""

import importlib
import sys
import time
from contextlib import contextmanager
from pathlib import Path
from unittest.mock import Mock

import pytest
from tinkerforge.ip_connection import IPConnection

from service import bricklet_discovery_service

# The enumeration measured on the robot. Delivered out of order on purpose.
# Position is the character the library passes through; the HAT Brick reports the
# letter of its place in the stack ("i") and is recognised as the carrier by its
# missing parent, not by an empty position - measured on the robot, and the
# reason the fixture below used to disagree with the hardware.
_MEASURED_DEVICES = (
    ("2jtj", "2iLa", "h", 2157),
    ("2iLa", "0", "i", 111),
    ("2dye", "2iLa", "a", 282),
    ("255m", "2iLa", "f", 282),
    ("27FV", "2iLa", "d", 296),
    ("2h4Z", "2iLa", "c", 2157),
    ("2dxM", "2iLa", "b", 282),
    ("2h4a", "2iLa", "g", 2157),
)

_EXPECTED_ORDER = (
    "2dye",
    "2dxM",
    "2h4Z",
    "27FV",
    "255m",
    "2h4a",
    "2jtj",
    "2iLa",
)

_EXPECTED_BY_UID = {
    "2dye": ("RGB LED Button Bricklet", "a", "2iLa", 282),
    "2dxM": ("RGB LED Button Bricklet", "b", "2iLa", 282),
    "2h4Z": ("Servo Bricklet 2.0", "c", "2iLa", 2157),
    "27FV": ("Solid State Relay Bricklet 2.0", "d", "2iLa", 296),
    "255m": ("RGB LED Button Bricklet", "f", "2iLa", 282),
    "2h4a": ("Servo Bricklet 2.0", "g", "2iLa", 2157),
    "2jtj": ("Servo Bricklet 2.0", "h", "2iLa", 2157),
    "2iLa": ("HAT Brick", "", "0", 111),
}


class _StubConnection:
    """Stands in for IPConnection. enumerate() delivers the scripted devices."""

    def __init__(self, devices, enumeration_type=None):
        self._devices = devices
        # Looked up at call time. A default argument would be bound when this
        # module is imported, which can be while a sibling test's stand-in
        # still occupies sys.modules. The service imports IPConnection later,
        # the callback ids then disagree, and enumerate() delivers nothing.
        if enumeration_type is None:
            enumeration_type = IPConnection.ENUMERATION_TYPE_CONNECTED
        self._enumeration_type = enumeration_type
        self.callbacks = {}
        self.enumerate_calls = 0

    def register_callback(self, callback_id, function):
        self.callbacks[callback_id] = function

    def enumerate(self):
        self.enumerate_calls += 1
        callback = self.callbacks.get(IPConnection.CALLBACK_ENUMERATE)
        if callback is None:
            return
        for uid, connected_uid, position, identifier in self._devices:
            callback(
                uid,
                connected_uid,
                position,
                (2, 0, 0),
                (2, 0, 0),
                identifier,
                self._enumeration_type,
            )

    def disconnect_link(self):
        callback = self.callbacks[IPConnection.CALLBACK_DISCONNECTED]
        callback(IPConnection.DISCONNECT_REASON_ERROR)


def _ip_connection_stand_in_installed() -> bool:
    """True when sys.modules holds a fake tinkerforge.ip_connection.

    tests/integration/test_motor_current.py assigns that stand-in at import
    time and only removes it after its own tests. Pytest imports this module
    during collection, so the IPConnection name above can be the stand-in's
    MagicMock while get_connected_bricklets() later imports the real class.
    """
    module = sys.modules.get("tinkerforge.ip_connection")
    if module is None:
        return False
    if isinstance(getattr(module, "IPConnection", None), Mock):
        return True
    return not hasattr(module, "get_device_display_name")


def _take_tinkerforge_stand_in() -> dict | None:
    if not _ip_connection_stand_in_installed():
        return None
    stolen = {}
    for name in list(sys.modules):
        if name == "tinkerforge" or name.startswith("tinkerforge."):
            stolen[name] = sys.modules.pop(name)
    return stolen


def _restore_tinkerforge_stand_in(stolen: dict) -> None:
    for name in list(sys.modules):
        if name == "tinkerforge" or name.startswith("tinkerforge."):
            sys.modules.pop(name, None)
    sys.modules.update(stolen)


@pytest.fixture(autouse=True)
def _isolated_discovery_state(monkeypatch):
    bricklet_discovery_service._reset_state()
    # The production deadline waits out the measured settle. The stub delivers
    # devices inside enumerate(), so the suite does not sleep that long.
    monkeypatch.setattr(bricklet_discovery_service, "ENUMERATE_DEADLINE_SECONDS", 0.0)
    # Share the library the service imports inside _on_enumerate. Restoring the
    # stand-in afterwards keeps a sibling module that installed it working.
    stolen = _take_tinkerforge_stand_in()
    try:
        ip_connection = importlib.import_module("tinkerforge.ip_connection")
        monkeypatch.setattr(
            sys.modules[__name__], "IPConnection", ip_connection.IPConnection
        )
        yield
    finally:
        bricklet_discovery_service._reset_state()
        if stolen is not None:
            _restore_tinkerforge_stand_in(stolen)


def _use_connection(monkeypatch, connection):
    monkeypatch.setattr(bricklet_discovery_service, "_get_tf_ipcon", lambda: connection)


def test_measured_devices_keep_name_uid_port_and_order(monkeypatch):
    _use_connection(monkeypatch, _StubConnection(_MEASURED_DEVICES))

    bricklets = bricklet_discovery_service.get_connected_bricklets()

    assert [device["uid"] for device in bricklets] == list(_EXPECTED_ORDER)
    for device in bricklets:
        name, port, parent_uid, identifier = _EXPECTED_BY_UID[device["uid"]]
        assert set(device) == {
            "name",
            "uid",
            "port",
            "parentUid",
            "deviceIdentifier",
        }
        assert device["name"] == name
        assert device["port"] == port
        assert device["parentUid"] == parent_uid
        assert device["deviceIdentifier"] == identifier


def test_carrier_without_a_parent_board_reports_no_port(monkeypatch):
    """The measured shape of the HAT Brick: no parent, but a position letter.

    Before this, the position was reported as a port and the client named a board
    that does not exist ("Port I on board 0").
    """
    connection = _StubConnection((("2iLa", "0", "i", 111),))
    _use_connection(monkeypatch, connection)

    bricklets = bricklet_discovery_service.get_connected_bricklets()

    assert len(bricklets) == 1
    assert bricklets[0]["parentUid"] == "0"
    assert bricklets[0]["port"] == ""
    assert bricklets[0]["name"] == "HAT Brick"


def test_carrier_with_an_empty_parent_uid_also_reports_no_port(monkeypatch):
    connection = _StubConnection((("2iLa", "", "i", 111),))
    _use_connection(monkeypatch, connection)

    bricklets = bricklet_discovery_service.get_connected_bricklets()

    assert bricklets[0]["port"] == ""


def test_a_bricklet_keeps_the_port_of_its_parent_board(monkeypatch):
    """The rule must not swallow real ports: a parent board means a socket."""
    connection = _StubConnection((("2dye", "2iLa", "a", 282),))
    _use_connection(monkeypatch, connection)

    bricklets = bricklet_discovery_service.get_connected_bricklets()

    assert bricklets[0]["port"] == "a"
    assert bricklets[0]["parentUid"] == "2iLa"


def test_device_without_a_port_keeps_the_port_key(monkeypatch):
    connection = _StubConnection((("2iLa", "0", "\x00", 111),))
    _use_connection(monkeypatch, connection)

    bricklets = bricklet_discovery_service.get_connected_bricklets()

    assert len(bricklets) == 1
    assert "port" in bricklets[0]
    assert bricklets[0]["port"] == ""
    assert bricklets[0]["port"] is not None


def test_unreachable_daemon_raises_instead_of_returning_an_empty_list(monkeypatch):
    _use_connection(monkeypatch, None)

    with pytest.raises(bricklet_discovery_service.BrickdUnreachableError) as raised:
        bricklet_discovery_service.get_connected_bricklets()

    assert "unreachable" in str(raised.value).lower()


@contextmanager
def _tinkerforge_hidden(monkeypatch):
    """Make `import tinkerforge` fail until the block exits.

    The suite imports the package at module level, so blocking builtins.__import__
    is not enough: the module is already cached in sys.modules.
    """
    import builtins
    import sys

    hidden = {
        name: module
        for name, module in list(sys.modules.items())
        if name == "tinkerforge" or name.startswith("tinkerforge.")
    }
    for name in hidden:
        sys.modules.pop(name, None)
    real_import = builtins.__import__

    def _blocked(name, globals=None, locals=None, fromlist=(), level=0):
        if name == "tinkerforge" or name.startswith("tinkerforge."):
            raise ImportError("No module named 'tinkerforge'")
        return real_import(name, globals, locals, fromlist, level)

    monkeypatch.setattr(builtins, "__import__", _blocked)
    try:
        yield
    finally:
        sys.modules.update(hidden)


def test_missing_library_is_not_reported_as_an_unreachable_daemon(monkeypatch):
    """None from the connection helper is a missing library when the import fails."""
    _use_connection(monkeypatch, None)

    with _tinkerforge_hidden(monkeypatch):
        with pytest.raises(
            bricklet_discovery_service.BrickletLibraryMissingError
        ) as raised:
            bricklet_discovery_service.get_connected_bricklets()

        assert not isinstance(
            raised.value, bricklet_discovery_service.BrickdUnreachableError
        )
        assert "tinkerforge" in str(raised.value).lower()
        assert "unreachable" not in str(raised.value).lower()


def test_callback_setup_reports_a_missing_library_instead_of_an_unreachable_daemon(
    monkeypatch,
):
    """ImportError inside _ensure_callback must not become BrickdUnreachableError."""
    _use_connection(monkeypatch, _StubConnection(()))

    with _tinkerforge_hidden(monkeypatch):
        with pytest.raises(
            bricklet_discovery_service.BrickletLibraryMissingError
        ) as raised:
            bricklet_discovery_service.get_connected_bricklets()

        assert not isinstance(
            raised.value, bricklet_discovery_service.BrickdUnreachableError
        )
        assert "tinkerforge" in str(raised.value).lower()
        assert "unreachable" not in str(raised.value).lower()


def test_reachable_daemon_with_nothing_attached_returns_an_empty_list(monkeypatch):
    _use_connection(monkeypatch, _StubConnection(()))

    assert bricklet_discovery_service.get_connected_bricklets() == []


def test_repeat_call_reuses_the_list_until_the_connection_drops(monkeypatch):
    connection = _StubConnection(_MEASURED_DEVICES)
    _use_connection(monkeypatch, connection)

    first = bricklet_discovery_service.get_connected_bricklets()
    callback = connection.callbacks[IPConnection.CALLBACK_ENUMERATE]
    callback(
        "2dye",
        "2iLa",
        "a",
        (2, 0, 0),
        (2, 0, 0),
        282,
        IPConnection.ENUMERATION_TYPE_DISCONNECTED,
    )
    callback(
        "XYZ1",
        "2iLa",
        "e",
        (2, 0, 0),
        (2, 0, 0),
        282,
        IPConnection.ENUMERATION_TYPE_AVAILABLE,
    )
    second = bricklet_discovery_service.get_connected_bricklets()

    assert connection.enumerate_calls == 1
    assert [device["uid"] for device in first][0] == "2dye"
    assert "2dye" not in {device["uid"] for device in second}
    added = next(device for device in second if device["uid"] == "XYZ1")
    assert added["port"] == "e"
    assert added["name"] == "RGB LED Button Bricklet"

    connection.disconnect_link()
    bricklet_discovery_service.get_connected_bricklets()
    assert connection.enumerate_calls == 2


def test_request_is_bounded_by_the_deadline(monkeypatch):
    monkeypatch.setattr(bricklet_discovery_service, "ENUMERATE_DEADLINE_SECONDS", 0.2)
    _use_connection(monkeypatch, _StubConnection(()))

    started = time.monotonic()
    result = bricklet_discovery_service.get_connected_bricklets()
    elapsed = time.monotonic() - started

    assert result == []
    assert elapsed >= 0.15
    assert elapsed < 1.0


def test_host_resolution_is_not_copied():
    source = Path(bricklet_discovery_service.__file__).read_text(encoding="utf-8")

    assert "_get_tf_ipcon" in source
    assert "172.17.0.1" not in source
    assert "192.168.1.28" not in source
    assert "host.docker.internal" not in source
    assert "TINKERFORGE_HOST" not in source


def test_connected_route_wins_and_a_number_still_resolves(app, monkeypatch):
    _use_connection(
        monkeypatch,
        _StubConnection((("2iLa", "0", "\x00", 111), ("2dye", "2iLa", "A", 282))),
    )

    with app.test_client() as client:
        connected = client.get("/bricklet/connected")
        numbered = client.get("/bricklet/1")

    assert connected.status_code == 200
    payload = connected.get_json()
    assert set(payload) == {"bricklets"}
    hat = next(device for device in payload["bricklets"] if device["uid"] == "2iLa")
    button = next(device for device in payload["bricklets"] if device["uid"] == "2dye")
    assert set(hat) == {"name", "uid", "port", "parentUid", "deviceIdentifier"}
    assert hat["port"] == ""
    assert button["port"] == "a"
    assert '"port":""' in connected.get_data(as_text=True).replace(" ", "")

    assert numbered.status_code == 200
    numbered_body = numbered.get_json()
    assert set(numbered_body) == {"uid"}
    assert "bricklets" not in numbered_body


def test_unreachable_daemon_answers_503_rather_than_an_empty_list(app, monkeypatch):
    _use_connection(monkeypatch, None)

    with app.test_client() as client:
        response = client.get("/bricklet/connected")

    assert response.status_code == 503
    body = response.get_json()
    assert body.get("bricklets") is None
    assert "unreachable" in body["error"].lower()


def test_reachable_empty_enumeration_answers_200(app, monkeypatch):
    _use_connection(monkeypatch, _StubConnection(()))

    with app.test_client() as client:
        response = client.get("/bricklet/connected")

    assert response.status_code == 200
    assert response.get_json() == {"bricklets": []}
