"""Legacy microphone-array REST responses.

The Flask backend intentionally never imports pyusb, probes USB, or opens the
ReSpeaker. ``ros-audio-io`` is the single device owner. Live telemetry and all
tuning/LED control are ROS topics and ROS 2 parameters consumed through
rosbridge; these functions remain only for compatibility with older clients.
"""

from __future__ import annotations

import threading
from copy import deepcopy
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional

from seed_profiles import HardwareProfile, get_profile
from seed_profiles.edu_microphone_tuning import (
    MICROPHONE_LED_RING,
    MICROPHONE_TUNING,
)
from service.system_property_service import (
    MICROPHONE_DESIRED_STATE_KEY,
    get_property_value,
    get_variant,
    set_property,
    set_property_if_missing,
)

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

_DEFAULT_TUNING: Dict[str, Any] = dict(MICROPHONE_TUNING)

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
_DEFAULT_LED: Dict[str, Any] = dict(MICROPHONE_LED_RING)

_desired_state_lock = threading.RLock()


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


def _timestamp() -> str:
    return datetime.now(timezone.utc).isoformat()


def build_default_desired_state(profile: HardwareProfile) -> Dict[str, Any]:
    return {
        "parameters": dict(profile.microphone_tuning),
        "led_ring": deepcopy(_DEFAULT_LED),
        "preset": "Standard",
        "updatedAt": _timestamp(),
        "revision": 1,
    }


def seed_desired_state(profile: HardwareProfile | None = None) -> Dict[str, Any]:
    """Create the profile default if absent and return the stored document."""

    with _desired_state_lock:
        selected_profile = profile or get_profile(get_variant())
        set_property_if_missing(
            MICROPHONE_DESIRED_STATE_KEY,
            build_default_desired_state(selected_profile),
            "default",
        )
        return deepcopy(get_property_value(MICROPHONE_DESIRED_STATE_KEY))


class MicrophoneArrayService:
    """Database-backed desired state; this class has no hardware code path."""

    def __init__(self) -> None:
        self._lock = _desired_state_lock

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
            desired_state = self.get_desired_state()
            return {
                "preset": desired_state["preset"],
                "presets": self.list_presets(),
                "parameters": dict(desired_state["parameters"]),
                "led_ring": dict(desired_state["led_ring"]),
                **_legacy_facts(),
            }

    def get_desired_state(self) -> Dict[str, Any]:
        with self._lock:
            desired_state = get_property_value(MICROPHONE_DESIRED_STATE_KEY)
            if desired_state is None:
                desired_state = seed_desired_state()
            return deepcopy(desired_state)

    def update_tuning(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """Validate legacy input and persist the desired device-owner state."""

        if not isinstance(payload, dict):
            raise ValueError("Request body must be a JSON object")

        with self._lock:
            desired_state = self.get_desired_state()
            tuning_state = desired_state["parameters"]
            led_state = desired_state["led_ring"]

            if payload.get("preset") is not None:
                preset_name = _normalize_preset_name(str(payload["preset"]))
                if preset_name != "Custom":
                    tuning_state.update(PRESETS[preset_name])
                desired_state["preset"] = preset_name

            if payload.get("parameters") is not None:
                parameters = payload["parameters"]
                if not isinstance(parameters, dict):
                    raise ValueError("'parameters' must be an object")
                tuning_state.update(
                    {
                        str(name): _validate_param(str(name), value)
                        for name, value in parameters.items()
                    }
                )
                desired_state["preset"] = "Custom"

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
                    led_state["mode"] = mode
                if led.get("brightness") is not None:
                    brightness = int(led["brightness"])
                    if brightness < 0 or brightness > 31:
                        raise ValueError("LED brightness must be in [0, 31]")
                    led_state["brightness"] = brightness
                if led.get("color") is not None:
                    led_state["color"] = _parse_hex_color(str(led["color"]))
                if led.get("vad_led") is not None:
                    led_state["vad_led"] = int(bool(led["vad_led"]))

            desired_state["revision"] += 1
            desired_state["updatedAt"] = _timestamp()
            set_property(MICROPHONE_DESIRED_STATE_KEY, desired_state, source="command")
            return self.get_tuning()

    def reset_for_tests(self) -> None:
        """Retained for callers from before desired state moved to the database."""


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


def get_desired_state() -> Dict[str, Any]:
    return get_service().get_desired_state()


def update_tuning(payload: Dict[str, Any]) -> Dict[str, Any]:
    return get_service().update_tuning(payload)
