"""The register port speaks the same XVF-3000 transfers as ros_audio_io.tuning."""

from __future__ import annotations

import array
import builtins
import importlib
import struct
import sys
import types
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
VOICE_ASSISTANT_PKG = REPO_ROOT / "ros_packages" / "voice_assistant"
ROS_AUDIO_PACKAGE = REPO_ROOT / "ros_packages" / "ros_audio_io"
for package in (VOICE_ASSISTANT_PKG, ROS_AUDIO_PACKAGE):
    if str(package) not in sys.path:
        sys.path.insert(0, str(package))

from voice_assistant.live_registers import (  # noqa: E402
    PRODUCT_ID,
    REGISTER_MAP,
    VENDOR_ID,
    NullRegisterPort,
    XmosRegisterPort,
    open_register_port,
)

# USB setup-packet flags Tuning combines via usb.util.
_CTRL_IN = 0x80
_CTRL_OUT = 0x00
_CTRL_TYPE_VENDOR = 0x40
_CTRL_RECIPIENT_DEVICE = 0x00
_READ = _CTRL_IN | _CTRL_TYPE_VENDOR | _CTRL_RECIPIENT_DEVICE
_WRITE = _CTRL_OUT | _CTRL_TYPE_VENDOR | _CTRL_RECIPIENT_DEVICE


class RecordingDevice:
    """Records ctrl_transfer arguments and answers one control-in payload."""

    def __init__(self, response):
        self.calls = []
        self._response = response

    def ctrl_transfer(
        self,
        bm_request_type,
        b_request,
        w_value,
        w_index,
        data_or_length,
        timeout,
    ):
        self.calls.append(
            {
                "bm_request_type": bm_request_type,
                "b_request": b_request,
                "w_value": w_value,
                "w_index": w_index,
                "data_or_length": data_or_length,
                "timeout": timeout,
            }
        )
        if bm_request_type & _CTRL_IN:
            return self._response
        return len(data_or_length)


class BoomDevice:
    def ctrl_transfer(self, *args, **kwargs):
        raise OSError("usb stalled")


def _packed_response(mantissa: int, exponent: int) -> array.array:
    """The array.array pyusb hands back. Tuning unpacks it with tobytes()."""
    return array.array("B", struct.pack(b"ii", mantissa, exponent))


def _read_command(name: str) -> tuple[int, int]:
    resid, offset, kind = REGISTER_MAP[name]
    command = 0x80 | offset
    if kind == "int":
        command |= 0x40
    return command, resid


def _install_usb(monkeypatch, find, dispose=None):
    core = types.ModuleType("usb.core")
    core.find = find
    util = types.ModuleType("usb.util")
    util.dispose_resources = dispose or (lambda device: None)
    usb = types.ModuleType("usb")
    usb.__path__ = []
    usb.core = core
    usb.util = util
    monkeypatch.setitem(sys.modules, "usb", usb)
    monkeypatch.setitem(sys.modules, "usb.core", core)
    monkeypatch.setitem(sys.modules, "usb.util", util)
    return util


@pytest.fixture
def pyusb_absent(monkeypatch):
    """Make ``import usb.core`` fail even if this host has pyusb."""
    real_import = builtins.__import__

    def blocked(name, globals=None, locals=None, fromlist=(), level=0):
        if name == "usb" or (isinstance(name, str) and name.startswith("usb.")):
            raise ImportError("pyusb is not installed")
        return real_import(name, globals, locals, fromlist, level)

    for key in ("usb", "usb.core", "usb.util"):
        monkeypatch.delitem(sys.modules, key, raising=False)
    monkeypatch.setattr(builtins, "__import__", blocked)


def test_register_map_matches_tuning_parameters(monkeypatch):
    """The three names used here are the same resid, offset and type as Tuning."""
    _install_usb(monkeypatch, find=lambda **kwargs: None)
    tuning = importlib.import_module("ros_audio_io.tuning")
    for name, spec in REGISTER_MAP.items():
        assert tuning.PARAMETERS[name][:3] == spec


def test_voice_activity_read_uses_the_tuning_transfer():
    command, resid = _read_command("VOICEACTIVITY")
    device = RecordingDevice(_packed_response(1, 0))
    port = XmosRegisterPort(device)
    assert port.read_voice_activity() is True
    assert device.calls == [
        {
            "bm_request_type": _READ,
            "b_request": 0,
            "w_value": command,
            "w_index": resid,
            "data_or_length": 8,
            "timeout": 100000,
        }
    ]
    silent = RecordingDevice(_packed_response(0, 0))
    assert XmosRegisterPort(silent).read_voice_activity() is False


def test_doa_read_decodes_the_integer_register():
    command, resid = _read_command("DOAANGLE")
    device = RecordingDevice(_packed_response(207, 0))
    port = XmosRegisterPort(device, timeout_ms=50)
    assert port.read_doa_angle() == 207
    call = device.calls[0]
    assert call["bm_request_type"] == _READ
    assert call["w_value"] == command
    assert call["w_index"] == resid
    assert call["data_or_length"] == 8
    assert call["timeout"] == 50


def test_vad_threshold_read_decodes_the_float_register():
    """GAMMAVAD_SR is mantissa * 2**exponent, the same layout Tuning uses."""
    command, resid = _read_command("GAMMAVAD_SR")
    assert command & 0x40 == 0
    device = RecordingDevice(_packed_response(15, 0))
    port = XmosRegisterPort(device)
    assert port.read_vad_threshold_db() == 15.0
    call = device.calls[0]
    assert call["bm_request_type"] == _READ
    assert call["b_request"] == 0
    assert call["w_value"] == command
    assert call["w_index"] == resid
    assert call["data_or_length"] == 8
    scaled = RecordingDevice(_packed_response(7, -1))
    assert XmosRegisterPort(scaled).read_vad_threshold_db() == 3.5


def test_vad_threshold_write_uses_the_tuning_payload():
    resid, offset, kind = REGISTER_MAP["GAMMAVAD_SR"]
    assert kind == "float"
    device = RecordingDevice(_packed_response(0, 0))
    port = XmosRegisterPort(device)
    assert port.write_vad_threshold_db(8.5) is True
    assert device.calls == [
        {
            "bm_request_type": _WRITE,
            "b_request": 0,
            "w_value": 0,
            "w_index": resid,
            "data_or_length": struct.pack(b"ifi", offset, 8.5, 0),
            "timeout": 100000,
        }
    ]


def test_usb_errors_return_a_value_and_do_not_raise():
    port = XmosRegisterPort(BoomDevice())
    assert port.read_voice_activity() is False
    assert port.read_doa_angle() is None
    assert port.read_vad_threshold_db() is None
    assert port.write_vad_threshold_db(8.5) is False
    closed = XmosRegisterPort(None)
    assert closed.read_voice_activity() is False
    assert closed.write_vad_threshold_db(1.0) is False


def test_null_register_port_reports_no_device_and_never_raises():
    port = NullRegisterPort()
    assert port.read_voice_activity() is False
    assert port.read_doa_angle() is None
    assert port.read_vad_threshold_db() is None
    assert port.write_vad_threshold_db(15.0) is False


def test_open_register_port_falls_back_when_pyusb_is_absent(pyusb_absent):
    port = open_register_port()
    assert isinstance(port, NullRegisterPort)
    assert XmosRegisterPort.open() is None


def test_open_register_port_falls_back_when_the_device_is_missing(monkeypatch):
    seen = {}

    def find(**kwargs):
        seen.update(kwargs)
        return None

    _install_usb(monkeypatch, find)
    assert XmosRegisterPort.open() is None
    port = open_register_port()
    assert isinstance(port, NullRegisterPort)
    assert seen == {"idVendor": VENDOR_ID, "idProduct": PRODUCT_ID}


def test_open_register_port_returns_the_array_when_it_is_present(monkeypatch):
    device = object()
    _install_usb(monkeypatch, find=lambda **kwargs: device)
    port = open_register_port()
    assert isinstance(port, XmosRegisterPort)


def test_open_returns_none_when_find_raises(monkeypatch):
    def find(**kwargs):
        raise OSError("no bus")

    _install_usb(monkeypatch, find)
    assert XmosRegisterPort.open() is None


def test_close_disposes_the_device(monkeypatch):
    disposed = []

    def dispose(device):
        disposed.append(device)

    _install_usb(monkeypatch, find=lambda **kwargs: None, dispose=dispose)
    device = object()
    port = XmosRegisterPort(device)
    port.close()
    assert disposed == [device]
    port.close()


def test_close_swallows_a_missing_library(pyusb_absent):
    XmosRegisterPort(object()).close()


def test_close_swallows_a_dispose_error(monkeypatch):
    def dispose(device):
        raise OSError("busy")

    _install_usb(monkeypatch, find=lambda **kwargs: None, dispose=dispose)
    XmosRegisterPort(object()).close()
