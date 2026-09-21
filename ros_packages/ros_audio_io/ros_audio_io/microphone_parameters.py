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

PRESETS: Dict[str, Dict[str, Any]] = {
    "Standard": {name: spec[3] for name, spec in PARAMETER_SPECS.items()},
    "Noisy Environment / ASR": {
        "AGCONOFF": 0,
        "AGCMAXGAIN": 31.6,
        "AGCDESIREDLEVEL": 0.005,
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


def readback_matches(expected: Any, actual: Any) -> bool:
    """Compare an XVF3000 register read-back with its requested value."""

    if isinstance(expected, float):
        return math.isclose(expected, float(actual), rel_tol=1e-5, abs_tol=1e-8)
    return expected == actual
