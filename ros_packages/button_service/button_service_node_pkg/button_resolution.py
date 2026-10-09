"""Map connected Tinkerforge devices onto button_id 1..3.

Cerebra fills its bricklet dropdowns from GET /bricklet/connected, keeps the
entries whose name starts with the device type, and orders them by port.
The button service and the Blockly client share that rule so a fixed slot
number can no longer hand a relay UID to the RGB LED Button API.

The device identifier is the fallback for an entry that has no name. A name
that says the device is something else wins, even when the identifier is the
button's.
"""

from __future__ import annotations

from dataclasses import dataclass

# RGB LED Button Bricklet. Prefer the name prefix; this is the identifier
# brickd reports for the same device (Tinkerforge device id 282).
RGB_LED_BUTTON_NAME_PREFIX = "RGB LED Button"
RGB_LED_BUTTON_DEVICE_IDENTIFIER = 282

BUTTON_IDS = (1, 2, 3)


@dataclass(frozen=True)
class ResolvedButton:
    """One button_id after port ordering.

    ``uid`` is None when that button cannot be used. ``reason`` is then the
    text callers log or return, and it is None for a usable button.
    """

    button_id: int
    uid: str | None
    port: str | None
    reason: str | None


def is_rgb_led_button(device: dict) -> bool:
    """True when the connected-device entry is an RGB LED Button Bricklet."""
    name = str(device.get("name") or "").strip()
    if name:
        return name.startswith(RGB_LED_BUTTON_NAME_PREFIX)
    try:
        return int(device.get("deviceIdentifier")) == RGB_LED_BUTTON_DEVICE_IDENTIFIER
    except (TypeError, ValueError):
        return False


def resolve_buttons(devices) -> dict[int, ResolvedButton]:
    """Assign button_id 1..3 from connected devices, ordered by port.

    Other device types are ignored. An RGB LED Button with no UID keeps its
    place in the port order and is reported as unavailable, so the buttons
    behind it do not shift into the empty slot. Missing ids above the devices
    that are present are unavailable too. This never raises.
    """
    buttons = [
        device
        for device in devices or []
        if isinstance(device, dict) and is_rgb_led_button(device)
    ]
    buttons.sort(key=_port_order)

    resolved: dict[int, ResolvedButton] = {}
    for index, button_id in enumerate(BUTTON_IDS):
        if index >= len(buttons):
            resolved[button_id] = ResolvedButton(
                button_id=button_id,
                uid=None,
                port=None,
                reason="no RGB LED Button Bricklet connected",
            )
            continue
        device = buttons[index]
        port = _normalize_port(device.get("port"))
        uid = _normalize_uid(device.get("uid"))
        if not uid:
            resolved[button_id] = ResolvedButton(
                button_id=button_id,
                uid=None,
                port=port or None,
                reason=f"RGB LED Button on port {port or '?'} has no UID",
            )
            continue
        resolved[button_id] = ResolvedButton(
            button_id=button_id,
            uid=uid,
            port=port or None,
            reason=None,
        )
    return resolved


def uid_for_button(devices, button_id: int) -> str:
    """Return the UID for button_id, or raise when that button has no device."""
    button_id = int(button_id)
    if button_id not in BUTTON_IDS:
        raise ValueError("button_id must be 1, 2, or 3")
    slot = resolve_buttons(devices)[button_id]
    if not slot.uid:
        raise RuntimeError(f"Button {button_id} is unavailable: {slot.reason}")
    return slot.uid


def _port_order(device: dict) -> tuple:
    port = _normalize_port(device.get("port"))
    # A device with no port sorts after the lettered sockets.
    return (port == "", port, _normalize_uid(device.get("uid")))


def _normalize_port(port) -> str:
    if port is None:
        return ""
    text = str(port).strip().lower()
    if not text or text[0] == "\x00":
        return ""
    return text


def _normalize_uid(uid) -> str:
    if not isinstance(uid, str):
        return ""
    return uid.strip()
