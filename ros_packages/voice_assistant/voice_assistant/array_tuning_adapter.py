"""Lazy access to XVF-3000 register reads via ros_audio_io.tuning (PC-102)."""

from __future__ import annotations

from typing import Any, Callable, Optional, Protocol


class ArrayTuningPort(Protocol):
    def read(self, name: str) -> Any: ...

    def write(self, name: str, value: Any) -> None: ...

    def is_voice(self) -> Any: ...


def open_array_tuning() -> Optional[ArrayTuningPort]:
    """Open the USB tuning port, or ``None`` when the device or package is missing."""
    try:
        from ros_audio_io.tuning import find as find_tuning
    except ImportError:
        return None
    try:
        return find_tuning()
    except Exception:
        return None


def tuning_is_voice(tuning: ArrayTuningPort) -> Callable[[], bool]:
    return lambda: bool(tuning.is_voice())


def tuning_read(tuning: ArrayTuningPort) -> Callable[[str], float]:
    return lambda name: float(tuning.read(name))


def tuning_write(tuning: ArrayTuningPort) -> Callable[[str, float], None]:
    def write(name: str, value: float) -> None:
        tuning.write(name, value)

    return write
