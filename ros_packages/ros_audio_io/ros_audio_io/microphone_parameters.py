"""Plain-Python definitions and validation for the microphone control surface."""

from __future__ import annotations

import math
from typing import Any, Dict

TUNABLE_PARAMETERS = (
    "AGCONOFF",
    "AGCMAXGAIN",
    "AGCDESIREDLEVEL",
    "AGCTIME",
    "STATNOISEONOFF",
    "NONSTATNOISEONOFF",
    "ECHOONOFF",
    "HPFONOFF",
    "STATNOISEONOFF_SR",
    "NONSTATNOISEONOFF_SR",
)

# name: (Python type, minimum, maximum, default)
PARAMETER_SPECS = {
    "AGCONOFF": (int, 0, 1, 1),
    "AGCMAXGAIN": (float, 1.0, 1000.0, 31.6),
    "AGCDESIREDLEVEL": (float, 1e-08, 0.99, 0.005),
    "AGCTIME": (float, 0.1, 1.0, 1.0),
    "STATNOISEONOFF": (int, 0, 1, 1),
    "NONSTATNOISEONOFF": (int, 0, 1, 1),
    "ECHOONOFF": (int, 0, 1, 1),
    "HPFONOFF": (int, 0, 3, 1),
    "STATNOISEONOFF_SR": (int, 0, 1, 1),
    "NONSTATNOISEONOFF_SR": (int, 0, 1, 1),
}

# AGCTIME is a ramp time-constant in seconds, but the register does not hold
# seconds: it holds the one-pole coefficient of that ramp at the processing
# block rate of the device,
#
#     coefficient = exp(-1 / (block rate * seconds))
#
# with a block rate of 16000 Hz / 256 samples = 62.5 Hz. Measured on
# 192.168.1.172 with a ReSpeaker Mic Array v2.0 attached:
#
#     write    model coefficient    read-back             residual
#     1.0 s    0.9841273201         0.9841422392055392    1.49e-05
#     0.5 s    0.9685065821         0.9685218567028642    1.53e-05
#
# Both read-backs follow the model to about 1.5e-05, so the register carries
# the requested quantity in a different unit - it is not a value the device
# refused. Read-backs are therefore converted back into seconds before they are
# compared or reported, and the parameter keeps its strict tolerance.
AGCTIME_BLOCK_RATE_HZ = 16000.0 / 256.0

# The 1.5e-05 coefficient residual grows when it is converted into seconds,
# because the conversion is steep near coefficient 1: at the top of the AGCTIME
# range it becomes 9.6e-04 s (1.0 s reads back as 1.00095 s), at the bottom
# 1.1e-05 s. This epsilon covers the whole range with margin while staying an
# order of magnitude below the 2e-02 s that a 2 % tolerance would have allowed
# at 1.0 s, so a write the device cannot store is still rejected.
AGCTIME_READBACK_EPSILON_SECONDS = 2e-3

# How far a float register may drift between the written and the read value.
# name: (relative tolerance, absolute tolerance)
#
# Everything is compared in the unit of the parameter, so the strict default
# holds for every register. AGCTIME keeps that strict relative tolerance and
# only widens the absolute epsilon, by the conversion residual documented
# above; widen a tolerance only with a measured read-back written down next
# to it.
DEFAULT_READBACK_TOLERANCE = (1e-5, 1e-8)
READBACK_TOLERANCES: Dict[str, tuple] = {
    "AGCTIME": (DEFAULT_READBACK_TOLERANCE[0], AGCTIME_READBACK_EPSILON_SECONDS),
}

PRESETS: Dict[str, Dict[str, Any]] = {
    "Standard": {name: spec[3] for name, spec in PARAMETER_SPECS.items()},
    "Noisy Environment / ASR": {
        "AGCONOFF": 0,
        "AGCMAXGAIN": 31.6,
        "AGCDESIREDLEVEL": 0.005,
        # The intended fast ramp of this preset. It was raised to 1.0 s while
        # the 0.9685 read-back of a 0.5 s write was read as seconds and taken
        # for a rejection; converted to seconds that read-back is 0.50025 s, so
        # the device does hold the value and the preset is faithful again.
        "AGCTIME": 0.5,
        "STATNOISEONOFF": 1,
        "NONSTATNOISEONOFF": 1,
        "ECHOONOFF": 1,
        "HPFONOFF": 2,
        "STATNOISEONOFF_SR": 1,
        "NONSTATNOISEONOFF_SR": 1,
    },
    "Loud Speaker Playback": {
        "AGCONOFF": 1,
        "AGCMAXGAIN": 15.8,
        "AGCDESIREDLEVEL": 0.005,
        "AGCTIME": 1.0,
        "STATNOISEONOFF": 1,
        "NONSTATNOISEONOFF": 1,
        "ECHOONOFF": 1,
        "HPFONOFF": 1,
        "STATNOISEONOFF_SR": 0,
        "NONSTATNOISEONOFF_SR": 0,
    },
    "Raw": {
        "AGCONOFF": 0,
        "AGCMAXGAIN": 1.0,
        "AGCDESIREDLEVEL": 0.005,
        "AGCTIME": 1.0,
        "STATNOISEONOFF": 0,
        "NONSTATNOISEONOFF": 0,
        "ECHOONOFF": 0,
        "HPFONOFF": 0,
        "STATNOISEONOFF_SR": 0,
        "NONSTATNOISEONOFF_SR": 0,
    },
    # Selecting Custom only labels the current individual register values.
    "Custom": {},
}

LED_MODES = ("off", "listen", "speak", "think", "spin", "trace", "mono")
LED_DEFAULTS = {
    "led_mode": "off",
    "led_brightness": 16,
    "led_color": "#000000",
    "vad_led": False,
}


def validate_tuning_parameter(name: str, value: Any) -> Any:
    """Coerce and range-check one XVF3000 tuning parameter."""

    if name not in PARAMETER_SPECS:
        raise ValueError(f"Unknown tuning parameter: {name}")

    value_type, minimum, maximum, _default = PARAMETER_SPECS[name]
    if value_type is int and (
        isinstance(value, bool)
        or not isinstance(value, int)
        or isinstance(value, float)
    ):
        raise ValueError(f"{name} must be an integer")
    if value_type is float and (
        isinstance(value, bool) or not isinstance(value, (int, float))
    ):
        raise ValueError(f"{name} must be a number")

    coerced = value_type(value)
    if isinstance(coerced, float) and not math.isfinite(coerced):
        raise ValueError(f"{name} must be finite")
    if coerced < minimum or coerced > maximum:
        raise ValueError(
            f"{name} must be in range [{minimum}, {maximum}]; got {coerced}"
        )
    return coerced


def validate_parameter(name: str, value: Any) -> Any:
    """Validate any ROS parameter exposed as the microphone control surface."""

    if name in PARAMETER_SPECS:
        return validate_tuning_parameter(name, value)
    if name == "preset":
        if not isinstance(value, str) or value not in PRESETS:
            raise ValueError(f"preset must be one of {tuple(PRESETS)}")
        return value
    if name == "led_mode":
        if not isinstance(value, str) or value not in LED_MODES:
            raise ValueError(f"led_mode must be one of {LED_MODES}")
        return value
    if name == "led_brightness":
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError("led_brightness must be an integer")
        if value < 0 or value > 31:
            raise ValueError("led_brightness must be in range [0, 31]")
        return value
    if name == "led_color":
        if not isinstance(value, str):
            raise ValueError("led_color must be a string in #RRGGBB form")
        color = value.upper()
        if (
            len(color) != 7
            or not color.startswith("#")
            or any(character not in "0123456789ABCDEF" for character in color[1:])
        ):
            raise ValueError("led_color must be in #RRGGBB form")
        return color
    if name == "vad_led":
        if not isinstance(value, bool):
            raise ValueError("vad_led must be a boolean")
        return value
    raise ValueError(f"Unknown microphone parameter: {name}")


def agctime_coefficient(seconds: float) -> float:
    """Convert an AGCTIME ramp time-constant into the register coefficient."""

    seconds = float(seconds)
    if math.isnan(seconds) or seconds <= 0.0:
        raise ValueError(f"AGCTIME must be a positive number of seconds; got {seconds}")
    return math.exp(-1.0 / (AGCTIME_BLOCK_RATE_HZ * seconds))


def agctime_seconds(coefficient: float) -> float:
    """Convert an AGCTIME register coefficient back into seconds."""

    coefficient = float(coefficient)
    if not 0.0 < coefficient < 1.0:
        raise ValueError(
            f"AGCTIME coefficient must be in range (0, 1); got {coefficient}"
        )
    return -1.0 / (AGCTIME_BLOCK_RATE_HZ * math.log(coefficient))


def parameter_from_readback(name: str, raw: Any) -> Any:
    """Convert one raw register read-back into the unit of its parameter.

    AGCTIME is the only register the device keeps in another unit; every other
    parameter is a plain number and is returned unchanged.
    """

    if name == "AGCTIME":
        return agctime_seconds(raw)
    return raw


def readback_tolerance(name: str) -> tuple:
    """Return the (relative, absolute) read-back tolerance of one parameter."""

    return READBACK_TOLERANCES.get(name, DEFAULT_READBACK_TOLERANCE)


def readback_matches(name: str, expected: Any, actual: Any) -> bool:
    """Compare a read-back, in the unit of the parameter, with its request."""

    if isinstance(expected, float):
        rel_tol, abs_tol = readback_tolerance(name)
        return math.isclose(expected, float(actual), rel_tol=rel_tol, abs_tol=abs_tol)
    return expected == actual


def readback_mismatch_reason(
    name: str, expected: Any, actual: Any, raw: Any = None
) -> str:
    """Describe a read-back the device did not reproduce within tolerance."""

    reason = f"{name} read-back {actual!r} != requested {expected!r}"
    details = []
    if raw is not None and raw != actual:
        details.append(f"device register {raw!r}")
    if isinstance(expected, float):
        rel_tol, abs_tol = readback_tolerance(name)
        details.append(f"tolerance: relative {rel_tol:g}, absolute {abs_tol:g}")
    if details:
        reason += " (" + "; ".join(details) + ")"
    return reason


def readback_unconvertible_reason(name: str, expected: Any, raw: Any, exc: Any) -> str:
    """Describe a read-back that is not a value of this parameter at all."""

    return (
        f"{name} read-back {raw!r} is not a value the device can hold for "
        f"requested {expected!r}: {exc}"
    )


def apply_tuning_values(device, values: Dict[str, Any]):
    """Write tuning values to the device and verify each read-back.

    Returns the read-backs collected so far, in the unit of each parameter,
    together with a failure reason, which is None when every value came back
    inside its tolerance. The device is anything offering ``write(name,
    value)`` and ``read(name)``.
    """

    readbacks: Dict[str, Any] = {}
    for name, value in values.items():
        try:
            device.write(name, value)
            raw = device.read(name)
        except Exception as exc:
            return readbacks, f"Device write/read-back failed: {exc}"
        try:
            actual = parameter_from_readback(name, raw)
        except ValueError as exc:
            # Nothing is recorded for a register whose content the documented
            # conversion cannot express, so no such value is reflected back.
            return readbacks, readback_unconvertible_reason(name, value, raw, exc)
        readbacks[name] = actual
        if not readback_matches(name, value, actual):
            return readbacks, readback_mismatch_reason(name, value, actual, raw)
    return readbacks, None
