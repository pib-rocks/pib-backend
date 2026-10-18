"""HID tuning for the microphone array, and the measurement that tells you whether it worked (PC-88).

Two things the array needs before its echo canceller is worth anything:

* the right switches - echo cancellation on, the non-linear canceller on, automatic
  gain *off* (AGC pulls up the noise floor in the pauses between words, and the
  transcriber then hears a room instead of silence);
* a reference. The canceller subtracts what it knows, and over USB the device is
  capture-only, so the playback signal has to arrive at the array's own analogue
  input. Whether that is wired on this robot is unknown until measured.

:func:`apply_tuning` writes the switches through the vendor control interface and
reports, per parameter, whether it was applied or refused. A refusal is normal - the
device may be unplugged, permission may be missing, or the firmware may not expose
the parameter - and it must never stop a voice session.

:func:`erle_db` is the measurement: echo return loss enhancement between what was
played and what was captured. Above roughly 10 dB the canceller is doing real work;
near 0 dB the "reference" is not reaching the device at all and the array is
hearing pib through the air, which is what barge-in confirmation has to survive.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

RESPEAKER_VID = 0x2886
RESPEAKER_PID = 0x0018

# Vendor control request used by the XVF-3000 firmware for its parameter block.
_CTRL_IN = 0xC0
_CTRL_OUT = 0x40
_PARAM_WRITE = 0x01
_PARAM_READ = 0x00

DEFAULT_TUNING: tuple[tuple[str, int], ...] = (
    ("ECHOONOFF", 1),  # echo cancellation on
    ("NLATTENONOFF", 1),  # non-linear canceller on: handles the loud tail
    ("AGCONOFF", 0),  # AGC off: silence stays silence
)


@dataclass(frozen=True)
class TuningOutcome:
    applied: tuple[str, ...] = ()
    refused: tuple[tuple[str, str], ...] = ()

    @property
    def ok(self) -> bool:
        return not self.refused


@dataclass
class HidDevice:
    """A pyusb device handle, kept behind an interface so tests can supply a fake."""

    device: object

    def write(self, parameter: str, value: int) -> None:
        payload = bytearray(64)
        payload[0] = _PARAM_WRITE
        name = parameter.encode("ascii")[:16]
        payload[1 : 1 + len(name)] = name
        payload[17] = value
        self.device.ctrl_transfer(  # type: ignore[attr-defined]
            _CTRL_OUT, 0, 0, 0, bytes(payload)
        )

    def read(self, parameter: str) -> int:
        payload = bytearray(64)
        payload[0] = _PARAM_READ
        name = parameter.encode("ascii")[:16]
        payload[1 : 1 + len(name)] = name
        raw = self.device.ctrl_transfer(  # type: ignore[attr-defined]
            _CTRL_IN, 0, 0, 0, bytes(payload), 64
        )
        return int(raw[0]) if raw else -1


def open_hid(vid: int = RESPEAKER_VID, pid: int = RESPEAKER_PID) -> HidDevice | None:
    """The array's vendor interface, or ``None`` when pyusb is missing or refuses."""
    try:
        import usb.core  # type: ignore[import-not-found]
    except ImportError:
        return None
    try:
        device = usb.core.find(idVendor=vid, idProduct=pid)
    except Exception:  # noqa: BLE001 - a missing udev rule raises here
        return None
    if device is None:
        return None
    return HidDevice(device=device)


def apply_tuning(
    hid: HidDevice | None,
    parameters: tuple[tuple[str, int], ...] = DEFAULT_TUNING,
) -> TuningOutcome:
    """Write every parameter, collect what was refused. Never raises."""
    if hid is None:
        return TuningOutcome(
            refused=tuple((name, "no HID device") for name, _ in parameters)
        )
    applied: list[str] = []
    refused: list[tuple[str, str]] = []
    for name, value in parameters:
        try:
            hid.write(name, value)
        except (
            Exception
        ) as exc:  # noqa: BLE001 - a refused control is a note, not a failure
            refused.append((name, type(exc).__name__))
        else:
            applied.append(name)
    return TuningOutcome(applied=tuple(applied), refused=tuple(refused))


def erle_db(reference: np.ndarray, recorded: np.ndarray) -> float:
    """Echo return loss enhancement: how much of the played signal the microphone did not hear.

    Both arrays are the same playback window, aligned. The result is
    ``10 * log10(played_energy / recorded_energy)``; a value near 0 dB means the
    capture contains the playback at full level, which is the "no reference wired"
    case. Negative values are clamped to 0 because they only mean the capture is
    louder than the reference, i.e. the person is talking.
    """
    played = np.asarray(reference, dtype=np.float64)
    captured = np.asarray(recorded, dtype=np.float64)
    if played.size == 0 or captured.size == 0:
        raise ValueError("both buffers must be non-empty")
    length = min(played.size, captured.size)
    played_energy = float(np.sum(played[:length] ** 2))
    captured_energy = float(np.sum(captured[:length] ** 2))
    if played_energy <= 0:
        raise ValueError("the reference is silent; nothing to cancel")
    if captured_energy <= 0:
        return float("inf")
    return max(0.0, 10.0 * math.log10(played_energy / captured_energy))
