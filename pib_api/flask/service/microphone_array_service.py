"""Legacy microphone-array REST responses.

The Flask backend intentionally never imports pyusb, probes USB, or opens the
ReSpeaker. ``ros-audio-io`` is the single device owner. Live telemetry and all
tuning/LED control are ROS topics and ROS 2 parameters consumed through
rosbridge; these functions remain only for compatibility with older clients.
"""

from __future__ import annotations

import threading
from copy import deepcopy
from typing import Any, Dict, List, Optional

VENDOR_ID = 0x2886
PRODUCT_ID = 0x0018
SIMULATION_REASON = "microphone array is owned by ros-audio-io"
CONTROL_SURFACE = "ROS 2 parameters via rosbridge"

# name: (type, maximum, minimum)
PARAMETERS = {
    "HPFONOFF": ("int", 3, 0),
    "AGCONOFF": ("int", 1, 0),
    "AGCMAXGAIN": ("float", 1000, 1),
    "AGCDESIREDLEVEL": ("float", 0.99, 1e-08),
    "AGCTIME": ("float", 1, 0.1),
    "STATNOISEONOFF": ("int", 1, 0),
    "NONSTATNOISEONOFF": ("int", 1, 0),
    "ECHOONOFF": ("int", 1, 0),
    "STATNOISEONOFF_SR": ("int", 1, 0),
    "NONSTATNOISEONOFF_SR": ("int", 1, 0),
}
TUNABLE_PARAMS = tuple(PARAMETERS)

_DEFAULT_TUNING: Dict[str, Any] = {
    "AGCONOFF": 1,
    "AGCMAXGAIN": 31.6,
    "AGCDESIREDLEVEL": 0.005,
    "AGCTIME": 1.0,
    "STATNOISEONOFF": 1,
    "NONSTATNOISEONOFF": 1,
    "ECHOONOFF": 1,
    "HPFONOFF": 1,
    "STATNOISEONOFF_SR": 1,
    "NONSTATNOISEONOFF_SR": 1,
}

PRESETS: Dict[str, Dict[str, Any]] = {
    "Standard": dict(_DEFAULT_TUNING),
    "Noisy Environment / ASR": {
        **_DEFAULT_TUNING,
        "AGCONOFF": 0,
        "AGCTIME": 0.5,
        "HPFONOFF": 2,
    },
    "Loud Speaker Playback": {
        **_DEFAULT_TUNING,
        "AGCMAXGAIN": 15.8,
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
    "Custom": dict(_DEFAULT_TUNING),
}
PRESET_ALIASES = {
    "Raw Pass-Through": "Raw",
    "raw": "Raw",
    "raw pass-through": "Raw",
}

LED_MODES = ("off", "listen", "speak", "think", "spin", "trace", "mono")
_DEFAULT_LED: Dict[str, Any] = {
    "mode": "off",
    "brightness": 16,
    "color": "#000000",
    "vad_led": 0,
}


def _legacy_facts() -> Dict[str, Any]:
    return {
        "legacy": True,
        "simulation": True,
        "simulation_reason": SIMULATION_REASON,
        "control_surface": CONTROL_SURFACE,
        "applied_to_device": False,
    }


def _normalize_preset_name(name: str) -> str:
    if name in PRESETS:
        return name
    stripped = name.strip()
    alias = PRESET_ALIASES.get(stripped) or PRESET_ALIASES.get(stripped.lower())
    if alias:
        return alias
    for preset in PRESETS:
        if preset.lower() == stripped.lower():
            return preset
    raise ValueError(f"Unknown preset: {name}")


def _parse_hex_color(color: str) -> str:
    normalized = color.strip().upper()
    if not normalized.startswith("#"):
        normalized = f"#{normalized}"
    if len(normalized) != 7 or any(
        character not in "0123456789ABCDEF" for character in normalized[1:]
    ):
        raise ValueError(f"Invalid color '{color}'; expected #RRGGBB")
    return normalized


def _validate_param(name: str, value: Any) -> Any:
    if name in ("DOAANGLE", "VOICEACTIVITY", "SPEECHDETECTED"):
        raise ValueError(f"{name} is read-only")
    if name not in PARAMETERS:
        raise ValueError(f"Unknown parameter: {name}")
    value_type, maximum, minimum = PARAMETERS[name]
    coerced = int(value) if value_type == "int" else float(value)
    if coerced < minimum or coerced > maximum:
        raise ValueError(f"{name} out of range [{minimum}, {maximum}]; got {coerced}")
    return coerced


class MicrophoneArrayService:
    """Compatibility-only state; this class has no hardware code path."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self.reset_for_tests()

    @property
    def is_simulation(self) -> bool:
        return True

    def list_presets(self) -> List[str]:
        return list(PRESETS)

    def health(self) -> Dict[str, Any]:
        with self._lock:
            return {
                **_legacy_facts(),
                "device_access": False,
                "owner": "ros-audio-io",
                "led_owner": "ros-audio-io",
                "led_control": CONTROL_SURFACE,
                "vendor_id": f"0x{VENDOR_ID:04x}",
                "product_id": f"0x{PRODUCT_ID:04x}",
                "note": "Live values come from ROS topics published by ros-audio-io.",
            }

    def get_telemetry(self) -> Dict[str, Any]:
        """Return no fabricated measurements from the retired backend owner."""

        with self._lock:
            return {
                "doa_angle": None,
                "voice_activity": None,
                "speech_detected": None,
                "audio_levels": [],
                **_legacy_facts(),
            }

    def get_tuning(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "preset": self._preset,
                "presets": self.list_presets(),
                "parameters": dict(self._tuning_state),
                "led_ring": dict(self._led_state),
                **_legacy_facts(),
            }

    def update_tuning(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """Validate legacy input and update only the explicitly simulated cache."""

        if not isinstance(payload, dict):
            raise ValueError("Request body must be a JSON object")

        with self._lock:
            if payload.get("preset") is not None:
                preset_name = _normalize_preset_name(str(payload["preset"]))
                if preset_name != "Custom":
                    self._tuning_state.update(PRESETS[preset_name])
                self._preset = preset_name

            if payload.get("parameters") is not None:
                parameters = payload["parameters"]
                if not isinstance(parameters, dict):
                    raise ValueError("'parameters' must be an object")
                self._tuning_state.update(
                    {
                        str(name): _validate_param(str(name), value)
                        for name, value in parameters.items()
                    }
                )
                self._preset = "Custom"

            if payload.get("led_ring") is not None:
                led = payload["led_ring"]
                if not isinstance(led, dict):
                    raise ValueError("'led_ring' must be an object")
                if led.get("mode") is not None:
                    mode = str(led["mode"]).lower()
                    if mode not in LED_MODES:
                        raise ValueError(
                            f"Unknown LED mode '{mode}'; expected one of {LED_MODES}"
                        )
                    self._led_state["mode"] = mode
                if led.get("brightness") is not None:
                    brightness = int(led["brightness"])
                    if brightness < 0 or brightness > 31:
                        raise ValueError("LED brightness must be in [0, 31]")
                    self._led_state["brightness"] = brightness
                if led.get("color") is not None:
                    self._led_state["color"] = _parse_hex_color(str(led["color"]))
                if led.get("vad_led") is not None:
                    self._led_state["vad_led"] = int(bool(led["vad_led"]))

            return self.get_tuning()

    def reset_for_tests(self) -> None:
        with self._lock:
            self._preset = "Standard"
            self._tuning_state = deepcopy(_DEFAULT_TUNING)
            self._led_state = deepcopy(_DEFAULT_LED)


_service: Optional[MicrophoneArrayService] = None
_service_lock = threading.Lock()


def get_service() -> MicrophoneArrayService:
    global _service
    with _service_lock:
        if _service is None:
            _service = MicrophoneArrayService()
        return _service


def get_telemetry() -> Dict[str, Any]:
    return get_service().get_telemetry()


def health() -> Dict[str, Any]:
    return get_service().health()


def get_tuning() -> Dict[str, Any]:
    return get_service().get_tuning()


def update_tuning(payload: Dict[str, Any]) -> Dict[str, Any]:
    return get_service().update_tuning(payload)
