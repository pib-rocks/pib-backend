"""Seeed ReSpeaker 4-Mic Array (USB XMOS XVF3000) driver & tuning service.

Falls back to an in-memory simulation when the USB device is absent or when
``MICROPHONE_ARRAY_SIMULATION`` is set.
"""

from __future__ import annotations

import logging
import os
import struct
import threading
from copy import deepcopy
from typing import Any, Dict, List, Optional

logger = logging.getLogger(__name__)

VENDOR_ID = 0x2886
PRODUCT_ID = 0x0018

# name: (id, offset, type, max, min, r/w, info…)
PARAMETERS: Dict[str, tuple] = {
    "HPFONOFF": (18, 27, "int", 3, 0, "rw", "High-pass Filter on microphone signals."),
    "AGCONOFF": (19, 0, "int", 1, 0, "rw", "Automatic Gain Control."),
    "AGCMAXGAIN": (19, 1, "float", 1000, 1, "rw", "Maximum AGC gain factor."),
    "AGCDESIREDLEVEL": (19, 2, "float", 0.99, 1e-08, "rw", "Target power level."),
    "AGCTIME": (19, 4, "float", 1, 0.1, "rw", "AGC ramp time-constant in seconds."),
    "STATNOISEONOFF": (19, 8, "int", 1, 0, "rw", "Stationary noise suppression."),
    "NONSTATNOISEONOFF": (19, 11, "int", 1, 0, "rw", "Non-stationary noise suppression."),
    "ECHOONOFF": (19, 14, "int", 1, 0, "rw", "Echo suppression."),
    "SPEECHDETECTED": (19, 22, "int", 1, 0, "ro", "Speech detection status."),
    "VOICEACTIVITY": (19, 32, "int", 1, 0, "ro", "VAD voice activity status."),
    "STATNOISEONOFF_SR": (19, 33, "int", 1, 0, "rw", "Stationary NS for ASR."),
    "NONSTATNOISEONOFF_SR": (19, 34, "int", 1, 0, "rw", "Non-stationary NS for ASR."),
    "DOAANGLE": (21, 0, "int", 359, 0, "ro", "DOA angle."),
}

TUNABLE_PARAMS = (
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

# Factory-style defaults for the Standard preset.
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
    "Custom": dict(_DEFAULT_TUNING),
}

# Accept the plan's longer alias for the Raw preset.
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

_DEFAULT_TELEMETRY: Dict[str, Any] = {
    "doa_angle": 180,
    "voice_activity": False,
    "speech_detected": False,
    "audio_levels": [0.05, 0.02, 0.02, 0.03, 0.02],
}


try:
    import usb.core  # type: ignore
    import usb.util  # type: ignore

    _USB_AVAILABLE = True
except ImportError:  # pragma: no cover - exercised when pyusb is absent
    usb = None  # type: ignore
    _USB_AVAILABLE = False


class ReSpeakerTuning:
    """Low-level USB vendor-control driver for XMOS DSP parameters."""

    TIMEOUT = 100000

    def __init__(self, dev: Any):
        self.dev = dev

    def write(self, name: str, value: Any) -> None:
        data = PARAMETERS[name]
        if data[5] == "ro":
            raise ValueError(f"{name} is read-only")
        param_id = data[0]
        if data[2] == "int":
            payload = struct.pack(b"iii", data[1], int(value), 1)
        else:
            payload = struct.pack(b"ifi", data[1], float(value), 0)
        self.dev.ctrl_transfer(
            usb.util.CTRL_OUT
            | usb.util.CTRL_TYPE_VENDOR
            | usb.util.CTRL_RECIPIENT_DEVICE,
            0,
            0,
            param_id,
            payload,
            self.TIMEOUT,
        )

    def read(self, name: str) -> Any:
        data = PARAMETERS[name]
        param_id = data[0]
        cmd = 0x80 | data[1]
        if data[2] == "int":
            cmd |= 0x40
        response = self.dev.ctrl_transfer(
            usb.util.CTRL_IN
            | usb.util.CTRL_TYPE_VENDOR
            | usb.util.CTRL_RECIPIENT_DEVICE,
            0,
            cmd,
            param_id,
            8,
            self.TIMEOUT,
        )
        unpacked = struct.unpack(b"ii", response.tobytes())
        if data[2] == "int":
            return unpacked[0]
        return unpacked[0] * (2.0 ** unpacked[1])


class PixelRing:
    """USB pixel-ring LED controller for ReSpeaker Mic Array v2.0."""

    TIMEOUT = 8000

    def __init__(self, dev: Any):
        self.dev = dev

    def write(self, cmd: int, data: Optional[List[int]] = None) -> None:
        if data is None:
            data = [0]
        self.dev.ctrl_transfer(
            usb.util.CTRL_OUT
            | usb.util.CTRL_TYPE_VENDOR
            | usb.util.CTRL_RECIPIENT_DEVICE,
            0,
            cmd,
            0x1C,
            data,
            self.TIMEOUT,
        )

    def off(self) -> None:
        self.write(1, [0, 0, 0, 0])

    def mono(self, color: int) -> None:
        self.write(
            1,
            [(color >> 16) & 0xFF, (color >> 8) & 0xFF, color & 0xFF, 0],
        )

    def listen(self) -> None:
        self.write(2)

    def speak(self) -> None:
        self.write(3)

    def think(self) -> None:
        self.write(4)

    def spin(self) -> None:
        self.write(5)

    def trace(self) -> None:
        self.write(0)

    def set_brightness(self, brightness: int) -> None:
        self.write(0x20, [int(brightness) & 0xFF])

    def set_vad_led(self, state: int) -> None:
        self.write(0x22, [int(state) & 0xFF])


def _force_simulation() -> bool:
    return os.getenv("MICROPHONE_ARRAY_SIMULATION", "").lower() in (
        "1",
        "true",
        "yes",
        "on",
    )


def _normalize_preset_name(name: str) -> str:
    if name in PRESETS:
        return name
    alias = PRESET_ALIASES.get(name) or PRESET_ALIASES.get(name.strip())
    if alias:
        return alias
    # Case-insensitive match against known presets.
    lowered = name.strip().lower()
    for preset in PRESETS:
        if preset.lower() == lowered:
            return preset
    alias = PRESET_ALIASES.get(lowered)
    if alias:
        return alias
    raise ValueError(f"Unknown preset: {name}")


def _parse_hex_color(color: str) -> int:
    value = color.strip().lstrip("#")
    if len(value) != 6:
        raise ValueError(f"Invalid color '{color}'; expected #RRGGBB")
    return int(value, 16)


def _validate_param(name: str, value: Any) -> Any:
    if name not in PARAMETERS:
        raise ValueError(f"Unknown parameter: {name}")
    meta = PARAMETERS[name]
    if meta[5] == "ro":
        raise ValueError(f"{name} is read-only")
    if name not in TUNABLE_PARAMS:
        raise ValueError(f"Parameter {name} is not exposed for tuning")
    if meta[2] == "int":
        coerced = int(value)
    else:
        coerced = float(value)
    minimum, maximum = meta[4], meta[3]
    if coerced < minimum or coerced > maximum:
        raise ValueError(
            f"{name} out of range [{minimum}, {maximum}]; got {coerced}"
        )
    return coerced


class MicrophoneArrayService:
    """Hardware / simulation façade for ReSpeaker telemetry and DSP tuning."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._dev: Any = None
        self._tuning: Optional[ReSpeakerTuning] = None
        self._pixel_ring: Optional[PixelRing] = None
        self._simulation = True
        self._preset = "Standard"
        self._tuning_state: Dict[str, Any] = dict(_DEFAULT_TUNING)
        self._led_state: Dict[str, Any] = dict(_DEFAULT_LED)
        self._telemetry_state: Dict[str, Any] = deepcopy(_DEFAULT_TELEMETRY)
        self._connect()

    def _connect(self) -> None:
        if _force_simulation() or not _USB_AVAILABLE:
            self._enter_simulation("forced or pyusb unavailable")
            return
        try:
            dev = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID)
        except Exception as exc:  # pragma: no cover - host USB stack errors
            self._enter_simulation(f"usb find failed: {exc}")
            return
        if dev is None:
            self._enter_simulation("ReSpeaker USB device not found")
            return
        self._dev = dev
        self._tuning = ReSpeakerTuning(dev)
        self._pixel_ring = PixelRing(dev)
        self._simulation = False
        logger.info("Microphone array: connected to ReSpeaker (0x%04x:0x%04x)", VENDOR_ID, PRODUCT_ID)
        try:
            self._apply_tuning_dict(self._tuning_state)
            self._apply_led_state(self._led_state)
        except Exception as exc:  # pragma: no cover
            logger.warning("Failed to sync initial tuning to hardware: %s", exc)

    def _enter_simulation(self, reason: str) -> None:
        self._simulation = True
        self._dev = None
        self._tuning = None
        self._pixel_ring = None
        logger.info("Microphone array: simulation mode (%s)", reason)

    @property
    def is_simulation(self) -> bool:
        return self._simulation

    def list_presets(self) -> List[str]:
        return list(PRESETS.keys())

    def get_telemetry(self) -> Dict[str, Any]:
        with self._lock:
            if self._simulation or self._tuning is None:
                return {
                    **deepcopy(self._telemetry_state),
                    "simulation": True,
                }
            try:
                doa = int(self._tuning.read("DOAANGLE"))
                vad = bool(self._tuning.read("VOICEACTIVITY"))
                speech = bool(self._tuning.read("SPEECHDETECTED"))
            except Exception as exc:
                logger.warning("Hardware telemetry read failed, using cache: %s", exc)
                return {
                    **deepcopy(self._telemetry_state),
                    "simulation": False,
                    "error": str(exc),
                }
            self._telemetry_state.update(
                {
                    "doa_angle": doa,
                    "voice_activity": vad,
                    "speech_detected": speech,
                }
            )
            return {
                "doa_angle": doa,
                "voice_activity": vad,
                "speech_detected": speech,
                "audio_levels": list(self._telemetry_state["audio_levels"]),
                "simulation": False,
            }

    def get_tuning(self) -> Dict[str, Any]:
        with self._lock:
            parameters = dict(self._tuning_state)
            if not self._simulation and self._tuning is not None:
                for name in TUNABLE_PARAMS:
                    try:
                        parameters[name] = self._tuning.read(name)
                    except Exception as exc:
                        logger.debug("Could not read %s from hardware: %s", name, exc)
                self._tuning_state = parameters
            return {
                "preset": self._preset,
                "presets": self.list_presets(),
                "parameters": parameters,
                "led_ring": dict(self._led_state),
                "simulation": self._simulation,
            }

    def update_tuning(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        if not isinstance(payload, dict):
            raise ValueError("Request body must be a JSON object")

        with self._lock:
            if "preset" in payload and payload["preset"] is not None:
                preset_name = _normalize_preset_name(str(payload["preset"]))
                if preset_name == "Custom":
                    # Selecting Custom keeps current parameters; just label them.
                    self._preset = "Custom"
                else:
                    self._apply_tuning_dict(PRESETS[preset_name])
                    self._preset = preset_name

            if "parameters" in payload and payload["parameters"] is not None:
                params = payload["parameters"]
                if not isinstance(params, dict):
                    raise ValueError("'parameters' must be an object")
                updates: Dict[str, Any] = {}
                for name, value in params.items():
                    updates[name] = _validate_param(str(name), value)
                self._apply_tuning_dict(updates)
                self._preset = "Custom"

            if "led_ring" in payload and payload["led_ring"] is not None:
                led = payload["led_ring"]
                if not isinstance(led, dict):
                    raise ValueError("'led_ring' must be an object")
                next_led = dict(self._led_state)
                if "mode" in led and led["mode"] is not None:
                    mode = str(led["mode"]).lower()
                    if mode not in LED_MODES:
                        raise ValueError(
                            f"Unknown LED mode '{mode}'; expected one of {LED_MODES}"
                        )
                    next_led["mode"] = mode
                if "brightness" in led and led["brightness"] is not None:
                    brightness = int(led["brightness"])
                    if brightness < 0 or brightness > 31:
                        raise ValueError("LED brightness must be in [0, 31]")
                    next_led["brightness"] = brightness
                if "color" in led and led["color"] is not None:
                    color = str(led["color"])
                    _parse_hex_color(color)  # validate
                    if not color.startswith("#"):
                        color = f"#{color}"
                    next_led["color"] = color.upper()
                if "vad_led" in led and led["vad_led"] is not None:
                    next_led["vad_led"] = 1 if bool(led["vad_led"]) else 0
                self._apply_led_state(next_led)

            return self.get_tuning()

    def _apply_tuning_dict(self, updates: Dict[str, Any]) -> None:
        for name, value in updates.items():
            coerced = _validate_param(name, value) if name in TUNABLE_PARAMS else value
            if name not in TUNABLE_PARAMS:
                continue
            self._tuning_state[name] = coerced
            if not self._simulation and self._tuning is not None:
                self._tuning.write(name, coerced)

    def _apply_led_state(self, led: Dict[str, Any]) -> None:
        self._led_state = {
            "mode": led.get("mode", "off"),
            "brightness": int(led.get("brightness", 16)),
            "color": led.get("color", "#000000"),
            "vad_led": int(led.get("vad_led", 0)),
        }
        if self._simulation or self._pixel_ring is None:
            return
        ring = self._pixel_ring
        ring.set_brightness(self._led_state["brightness"])
        ring.set_vad_led(self._led_state["vad_led"])
        mode = self._led_state["mode"]
        if mode == "off":
            ring.off()
        elif mode == "listen":
            ring.listen()
        elif mode == "speak":
            ring.speak()
        elif mode == "think":
            ring.think()
        elif mode == "spin":
            ring.spin()
        elif mode == "trace":
            ring.trace()
        elif mode == "mono":
            ring.mono(_parse_hex_color(self._led_state["color"]))

    def reset_for_tests(self) -> None:
        """Reset in-memory state; used by unit tests."""
        with self._lock:
            self._preset = "Standard"
            self._tuning_state = dict(_DEFAULT_TUNING)
            self._led_state = dict(_DEFAULT_LED)
            self._telemetry_state = deepcopy(_DEFAULT_TELEMETRY)
            self._enter_simulation("test reset")


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


def get_tuning() -> Dict[str, Any]:
    return get_service().get_tuning()


def update_tuning(payload: Dict[str, Any]) -> Dict[str, Any]:
    return get_service().update_tuning(payload)
