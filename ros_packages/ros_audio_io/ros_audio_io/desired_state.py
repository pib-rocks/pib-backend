"""Fetch and validate the persisted microphone-array desired state."""

from __future__ import annotations

import json
import os
from collections.abc import Mapping
from typing import Any, Dict
from urllib import error, request

from ros_audio_io.device_retry import next_retry_delay
from ros_audio_io.microphone_parameters import (
    LED_DEFAULTS,
    PARAMETER_SPECS,
    validate_parameter,
)

DEFAULT_FLASK_API_BASE_URL = "http://flask-app:5000"
DESIRED_STATE_PATH = "/system/microphone-array/desired-state"
FETCH_TIMEOUT_SECONDS = 3.0


class DesiredStateValidationError(ValueError):
    """Raised when a desired-state document is unsafe to apply."""


class DesiredStateApplyError(RuntimeError):
    """Raised when a validated state cannot be verified on the device."""


def build_desired_state_url(environ=None) -> str:
    """Build the desired-state URL from an injectable environment mapping."""

    source = os.environ if environ is None else environ
    base_url = source.get("FLASK_API_BASE_URL", DEFAULT_FLASK_API_BASE_URL)
    if not isinstance(base_url, str) or not base_url.strip():
        raise ValueError("FLASK_API_BASE_URL must be a non-empty string")
    return f"{base_url.rstrip('/')}{DESIRED_STATE_PATH}"


def _required_mapping(document: Mapping[str, Any], field: str) -> Mapping[str, Any]:
    if field not in document:
        raise DesiredStateValidationError(f"missing field: {field}")
    value = document[field]
    if not isinstance(value, Mapping):
        raise DesiredStateValidationError(f"{field} must be an object")
    return value


def validate_desired_state(document: Any) -> Dict[str, Any]:
    """Return a normalized document only when every device value is valid."""

    if not isinstance(document, Mapping):
        raise DesiredStateValidationError("document must be an object")

    parameters = _required_mapping(document, "parameters")
    unknown_parameters = set(parameters) - set(PARAMETER_SPECS)
    if unknown_parameters:
        unknown = sorted(unknown_parameters)[0]
        raise DesiredStateValidationError(f"unknown parameter: {unknown}")
    missing_parameters = set(PARAMETER_SPECS) - set(parameters)
    if missing_parameters:
        missing = sorted(missing_parameters)[0]
        raise DesiredStateValidationError(f"missing parameter: {missing}")

    normalized_parameters = {}
    for name in PARAMETER_SPECS:
        try:
            normalized_parameters[name] = validate_parameter(name, parameters[name])
        except ValueError as exc:
            raise DesiredStateValidationError(str(exc)) from exc

    led_ring = _required_mapping(document, "led_ring")
    expected_led_fields = {"mode", "brightness", "color", "vad_led"}
    unknown_led_fields = set(led_ring) - expected_led_fields
    if unknown_led_fields:
        unknown = sorted(unknown_led_fields)[0]
        raise DesiredStateValidationError(f"unknown led_ring field: {unknown}")
    missing_led_fields = expected_led_fields - set(led_ring)
    if missing_led_fields:
        missing = sorted(missing_led_fields)[0]
        raise DesiredStateValidationError(f"missing led_ring field: {missing}")

    vad_led = led_ring["vad_led"]
    if isinstance(vad_led, bool):
        normalized_vad_led = vad_led
    elif isinstance(vad_led, int) and vad_led in (0, 1):
        normalized_vad_led = bool(vad_led)
    else:
        raise DesiredStateValidationError("vad_led must be a boolean or 0/1")

    led_values = {
        "led_mode": led_ring["mode"],
        "led_brightness": led_ring["brightness"],
        "led_color": led_ring["color"],
        "vad_led": normalized_vad_led,
    }
    normalized_led = {}
    for name in LED_DEFAULTS:
        try:
            normalized_led[name] = validate_parameter(name, led_values[name])
        except ValueError as exc:
            raise DesiredStateValidationError(str(exc)) from exc

    if "preset" not in document:
        raise DesiredStateValidationError("missing field: preset")
    try:
        preset = validate_parameter("preset", document["preset"])
    except ValueError as exc:
        raise DesiredStateValidationError(str(exc)) from exc

    if "revision" not in document:
        raise DesiredStateValidationError("missing field: revision")
    revision = document["revision"]
    if isinstance(revision, bool) or not isinstance(revision, int) or revision < 1:
        raise DesiredStateValidationError("revision must be a positive integer")

    return {
        "parameters": normalized_parameters,
        "led_ring": normalized_led,
        "preset": preset,
        "revision": revision,
    }


def fetch_desired_state(url: str, timeout=FETCH_TIMEOUT_SECONDS, opener=None):
    """Fetch, decode and fully validate one desired-state document."""

    open_url = request.urlopen if opener is None else opener
    with open_url(url, timeout=timeout) as response:
        payload = response.read()
    return validate_desired_state(json.loads(payload))


def should_apply_revision(applied_revision, desired_revision) -> bool:
    """Return whether a fetched revision has not been applied to this device."""

    return applied_revision != desired_revision


def describe_desired_state_failure(exc: Exception) -> str:
    """Return one stable operator-facing reason for a reconciliation failure."""

    if isinstance(exc, DesiredStateValidationError):
        return "invalid desired state"
    if isinstance(exc, DesiredStateApplyError):
        return "desired state apply failure"
    if isinstance(exc, (json.JSONDecodeError, UnicodeDecodeError)):
        return "invalid desired state response"
    if isinstance(exc, (error.URLError, OSError, TimeoutError)):
        return "backend unreachable"
    return "desired state reconciliation failure"


def desired_state_failure(exc: Exception, attempt: int) -> tuple[str, float]:
    """Return the named reason and existing bounded retry delay."""

    return describe_desired_state_failure(exc), next_retry_delay(attempt)
