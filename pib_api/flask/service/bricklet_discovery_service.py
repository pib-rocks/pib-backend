"""Read the Bricklets currently attached to the robot.

The list comes from a Tinkerforge enumeration on the connection that
diagnostics_service already keeps. Host selection stays in
diagnostics_service._get_tf_ipcon(); this module does not keep a second copy.
"""

import threading
from typing import Any, Dict, List

from service.diagnostics_service import _get_tf_ipcon

# A full enumeration on the robot settles in about four seconds. The deadline
# sits just above that: long enough for the devices to answer, short enough
# that a request cannot hang.
ENUMERATE_DEADLINE_SECONDS = 5.0

_PUBLIC_KEYS = ("name", "uid", "port", "parentUid", "deviceIdentifier")

_devices: Dict[str, Dict[str, Any]] = {}
_devices_lock = threading.Lock()
_enumerate_lock = threading.Lock()
_state_lock = threading.Lock()
_callback_connection: Any = None
_generation = 0
_session = 0
_list_current = False


class BrickdUnreachableError(Exception):
    """brickd could not be reached. This is not an empty device list."""


def get_connected_bricklets() -> List[Dict[str, Any]]:
    """Return the devices brickd reports, ordered by port.

    An empty list means the daemon answered and nothing is attached.
    BrickdUnreachableError means the daemon could not be reached.
    """
    ipcon = _get_tf_ipcon()
    if ipcon is None:
        raise BrickdUnreachableError("Tinkerforge daemon is unreachable")

    with _enumerate_lock:
        try:
            _ensure_callback(ipcon)
        except Exception as error:
            raise BrickdUnreachableError("Tinkerforge daemon is unreachable") from error
        if _is_list_current():
            return _snapshot()

        started_session = _begin_enumeration()
        try:
            ipcon.enumerate()
        except Exception as error:
            raise BrickdUnreachableError("Tinkerforge daemon is unreachable") from error
        # enumerate() has no completion signal. Wait out the deadline and
        # answer with whatever the callback has collected by then.
        _wait_for_enumeration(ENUMERATE_DEADLINE_SECONDS)
        _finish_enumeration(started_session)
        return _snapshot()


def _reset_state() -> None:
    """Drop the cached list. Tests use this so cases do not share a connection."""
    global _callback_connection, _generation, _session, _list_current
    with _devices_lock:
        _devices.clear()
    with _state_lock:
        _callback_connection = None
        _generation = 0
        _session = 0
        _list_current = False


def _ensure_callback(ipcon: Any) -> None:
    global _callback_connection, _list_current
    if _callback_connection is ipcon:
        return

    from tinkerforge.ip_connection import IPConnection

    with _devices_lock:
        _devices.clear()
    with _state_lock:
        _list_current = False
        _callback_connection = ipcon
    ipcon.register_callback(IPConnection.CALLBACK_ENUMERATE, _on_enumerate)
    ipcon.register_callback(IPConnection.CALLBACK_DISCONNECTED, _on_disconnected)


def _on_disconnected(_reason: Any) -> None:
    """A dropped connection may have missed plug events. The next read re-enumerates."""
    global _session, _list_current
    with _state_lock:
        _session += 1
        _list_current = False


def _is_carrier(parent_uid: str) -> bool:
    """A device the enumeration reports without a parent board.

    Tinkerforge reports ``connected_uid`` as ``"0"`` (and occasionally as an
    empty string) for a Brick that sits on nothing - the HAT Brick on this robot.
    Such a device has no port; the position it reports is its place in the stack,
    not a socket. Measured on the robot: the HAT Brick 2iLa answers with
    connected_uid "0" and position "i".
    """
    return parent_uid in ("", "0")


def _on_enumerate(
    uid: Any,
    connected_uid: Any,
    position: Any,
    _hardware_version: Any,
    _firmware_version: Any,
    device_identifier: Any,
    enumeration_type: Any,
) -> None:
    from tinkerforge.ip_connection import IPConnection, get_device_display_name

    normalized_uid = _normalize_uid(uid)
    if enumeration_type == IPConnection.ENUMERATION_TYPE_DISCONNECTED:
        with _devices_lock:
            _devices.pop(normalized_uid, None)
        return
    if enumeration_type not in (
        IPConnection.ENUMERATION_TYPE_AVAILABLE,
        IPConnection.ENUMERATION_TYPE_CONNECTED,
    ):
        return

    parent_uid = _normalize_uid(connected_uid)
    device = {
        "name": get_device_display_name(device_identifier),
        "uid": normalized_uid,
        # A carrier has no port, and an empty port means "no port" throughout
        # this contract, so it is decided here once rather than left for every
        # client to work out from parentUid.
        "port": "" if _is_carrier(parent_uid) else _normalize_port(position),
        "parentUid": parent_uid,
        "deviceIdentifier": int(device_identifier),
        "_generation": _generation,
    }
    with _devices_lock:
        _devices[normalized_uid] = device


def _begin_enumeration() -> int:
    global _generation, _list_current
    with _state_lock:
        _generation += 1
        _list_current = False
        return _session


def _finish_enumeration(started_session: int) -> None:
    """Keep devices reported for this enumeration, and mark the list current.

    A disconnect during the wait leaves the list stale so the next request
    enumerates again instead of serving a half-read.
    """
    with _devices_lock:
        stale_uids = [
            uid
            for uid, device in _devices.items()
            if device.get("_generation") != _generation
        ]
        for uid in stale_uids:
            del _devices[uid]
    with _state_lock:
        global _list_current
        _list_current = _session == started_session


def _wait_for_enumeration(deadline: float) -> None:
    if deadline <= 0:
        return
    threading.Event().wait(deadline)


def _is_list_current() -> bool:
    with _state_lock:
        return _list_current


def _snapshot() -> List[Dict[str, Any]]:
    with _devices_lock:
        devices = [
            {key: device[key] for key in _PUBLIC_KEYS} for device in _devices.values()
        ]
    devices.sort(
        key=lambda device: (device["port"] == "", device["port"], device["uid"])
    )
    return devices


def _normalize_port(position: Any) -> str:
    """Return the lowercase port letter, or '' when the device has no position.

    The key is always present. A missing port and a null port are different
    things to the client: the carrier board reports no position.
    """
    if position is None:
        return ""
    if isinstance(position, (bytes, bytearray)):
        if position in (b"", b"\x00"):
            return ""
        position = position.decode("ascii", errors="ignore")
    elif isinstance(position, int):
        if position == 0:
            return ""
        position = chr(position)
    text = str(position)
    if text == "" or text[0] == "\x00":
        return ""
    return text.lower()


def _normalize_uid(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, (bytes, bytearray)):
        value = value.split(b"\x00", 1)[0].decode("ascii", errors="ignore")
    text = str(value)
    nul = text.find("\x00")
    if nul >= 0:
        text = text[:nul]
    return text
