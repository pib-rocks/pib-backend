"""Register port for the ReSpeaker microphone array (XMOS XVF-3000).

The transfer shape matches ``ros_audio_io.tuning.Tuning``. ``usb.core`` is
imported only when a device is opened or closed, so this module imports on
a machine that does not have pyusb. A USB hiccup returns a value and does
not raise: it must not kill the voice node.
"""

from __future__ import annotations

import struct
from typing import Protocol

VENDOR_ID = 0x2886
PRODUCT_ID = 0x0018

# (resid, offset, type). The same three values as tuning.PARAMETERS.
REGISTER_MAP = {
    "VOICEACTIVITY": (19, 32, "int"),
    "GAMMAVAD_SR": (19, 39, "float"),
    "DOAANGLE": (21, 0, "int"),
}

# USB setup-packet flags, the same values usb.util combines in Tuning.
_CTRL_IN = 0x80
_CTRL_OUT = 0x00
_CTRL_TYPE_VENDOR = 0x40
_CTRL_RECIPIENT_DEVICE = 0x00
_READ_REQUEST = _CTRL_IN | _CTRL_TYPE_VENDOR | _CTRL_RECIPIENT_DEVICE
_WRITE_REQUEST = _CTRL_OUT | _CTRL_TYPE_VENDOR | _CTRL_RECIPIENT_DEVICE
_READ_LENGTH = 8


class RegisterPort(Protocol):
    """The voice node reads the array through this, and only this."""

    def read_voice_activity(self) -> bool:
        """True when the hardware voice-activity register is set."""

    def read_doa_angle(self) -> int | None:
        """Direction of arrival in degrees, or None when it cannot be read."""

    def read_vad_threshold_db(self) -> float | None:
        """Current ``GAMMAVAD_SR`` threshold in dB, or None."""

    def write_vad_threshold_db(self, value: float) -> bool:
        """Write the threshold. True when the write went out."""


class XmosRegisterPort:
    """pyusb port. The control transfer matches ``Tuning.read`` / ``Tuning.write``."""

    def __init__(self, device=None, timeout_ms: int = 100000) -> None:
        self._device = device
        self._timeout_ms = timeout_ms

    @classmethod
    def open(cls) -> XmosRegisterPort | None:
        """The array, or None when pyusb or the device is missing."""
        try:
            import usb.core
        except ImportError:
            return None
        try:
            device = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID)
        except Exception:
            return None
        if not device:
            return None
        return cls(device)

    def close(self) -> None:
        device = self._device
        self._device = None
        if device is None:
            return
        try:
            import usb.util

            usb.util.dispose_resources(device)
        except Exception:
            return

    def read_voice_activity(self) -> bool:
        try:
            value = self._read("VOICEACTIVITY")
        except Exception:
            return False
        return bool(value)

    def read_doa_angle(self) -> int | None:
        try:
            value = self._read("DOAANGLE")
        except Exception:
            return None
        if isinstance(value, bool) or not isinstance(value, int):
            return None
        return value

    def read_vad_threshold_db(self) -> float | None:
        try:
            value = self._read("GAMMAVAD_SR")
        except Exception:
            return None
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return None
        return float(value)

    def write_vad_threshold_db(self, value: float) -> bool:
        try:
            self._write("GAMMAVAD_SR", float(value))
        except Exception:
            return False
        return True

    def _read(self, name: str) -> int | float:
        resid, offset, kind = REGISTER_MAP[name]
        command = 0x80 | offset
        if kind == "int":
            command |= 0x40
        if self._device is None:
            raise OSError("microphone array is not open")
        response = self._device.ctrl_transfer(
            _READ_REQUEST,
            0,
            command,
            resid,
            _READ_LENGTH,
            self._timeout_ms,
        )
        raw = _response_bytes(response)
        mantissa, exponent = struct.unpack(b"ii", raw)
        if kind == "int":
            return mantissa
        return mantissa * (2.0**exponent)

    def _write(self, name: str, value: float) -> None:
        resid, offset, kind = REGISTER_MAP[name]
        if kind == "int":
            payload = struct.pack(b"iii", offset, int(value), 1)
        else:
            payload = struct.pack(b"ifi", offset, float(value), 0)
        if self._device is None:
            raise OSError("microphone array is not open")
        self._device.ctrl_transfer(
            _WRITE_REQUEST,
            0,
            0,
            resid,
            payload,
            self._timeout_ms,
        )


class NullRegisterPort:
    """Stand-in for a missing array. Every call returns and none of them raise."""

    def read_voice_activity(self) -> bool:
        return False

    def read_doa_angle(self) -> int | None:
        return None

    def read_vad_threshold_db(self) -> float | None:
        return None

    def write_vad_threshold_db(self, value: float) -> bool:
        return False


def open_register_port() -> RegisterPort:
    """The array when it can be opened, otherwise a port that reports no device."""
    port = XmosRegisterPort.open()
    if port is None:
        return NullRegisterPort()
    return port


def _response_bytes(response: object) -> bytes:
    """Bytes of a control-in payload. pyusb returns an array with ``tobytes``."""
    to_bytes = getattr(response, "tobytes", None)
    if callable(to_bytes):
        raw = to_bytes()
    else:
        raw = response
    if isinstance(raw, bytes):
        return raw
    return bytes(raw)
