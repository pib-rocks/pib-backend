import logging
import time
from typing import Optional

import rclpy
from rclpy.node import Node

from button_service.srv import ReadButton, WaitForButton, SetButtonManualOverride
from button_service_node_pkg.button_resolution import uid_for_button
from datatypes.msg import ButtonColor
from pib_api_client import bricklet_client

_node: Optional[Node] = None
_set_button_color_publisher = None


def _ensure_node() -> Node:
    global _node
    global _set_button_color_publisher
    if not rclpy.ok():
        rclpy.init(args=None)
    if _node is None:
        _node = rclpy.create_node("pib_button_blockly_client")
        _set_button_color_publisher = _node.create_publisher(
            ButtonColor, "set_button_color", 10
        )
    return _node


def _call(service_type, service_name: str, request, timeout_sec: float = 10.0):
    node = _ensure_node()
    client = node.create_client(service_type, service_name)

    if not client.wait_for_service(timeout_sec=timeout_sec):
        raise RuntimeError(f"Service {service_name} not available")

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)

    if not future.done():
        raise TimeoutError(f"Service call to {service_name} timed out")

    result = future.result()
    if result is None:
        raise RuntimeError(f"Service call to {service_name} failed")

    if hasattr(result, "success") and not result.success:
        raise RuntimeError(result.message)

    return result


def set_button_color(
    button_id: int,
    red: int,
    green: int,
    blue: int,
    *,
    sticky: bool = True,
    clear: bool = False,
) -> None:
    node = _ensure_node()
    uid = _button_id_to_uid(button_id)

    msg = ButtonColor()
    msg.bricklet_uid = str(uid)
    msg.red = int(red)
    msg.green = int(green)
    msg.blue = int(blue)
    msg.sticky = bool(sticky)
    msg.clear = bool(clear)

    logging.info(
        f"Publishing set_button_color for button_id={int(button_id)} uid={uid} "
        f"rgb=({int(red)},{int(green)},{int(blue)}) sticky={bool(sticky)} clear={bool(clear)}"
    )

    global _set_button_color_publisher
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        if _set_button_color_publisher.get_subscription_count() > 0:
            break
        rclpy.spin_once(node, timeout_sec=0.1)
    else:
        logging.warning(
            "No subscribers on set_button_color after 3s; publishing anyway"
        )

    _set_button_color_publisher.publish(msg)
    # Give DDS a moment to deliver; Blockly programs often exit immediately.
    rclpy.spin_once(node, timeout_sec=0.25)


def _load_connected_devices() -> list:
    """Same source as the button service: GET /bricklet/connected."""
    try:
        successful, payload = bricklet_client.get_connected_bricklets()
    except Exception as exc:
        raise RuntimeError(
            f"Failed to resolve button UID via /bricklet/connected: {exc}"
        ) from exc
    if not successful or not isinstance(payload, dict):
        raise RuntimeError("Failed to resolve button UID via /bricklet/connected")
    devices = payload.get("bricklets") or []
    if not isinstance(devices, list):
        raise RuntimeError("Failed to resolve button UID via /bricklet/connected")
    return devices


def _button_id_to_uid(button_id: int) -> str:
    """
    Resolve button_id (1..3) to a bricklet UID with the same type-and-port
    rule as the button service.
    """
    return uid_for_button(_load_connected_devices(), button_id)


def set_button_manual_override(button_id: int, red: int, green: int, blue: int) -> None:
    # Backwards-compatible shim: manual_override is now just a sticky set via topic.
    set_button_color(button_id, red, green, blue, sticky=True, clear=False)


def is_button_pressed(button_id: int) -> bool:
    req = ReadButton.Request()
    req.button_id = int(button_id)
    res = _call(ReadButton, "/tf_button/read", req)
    return bool(res.pressed)


def is_button_switched_on(button_id: int) -> bool:
    req = ReadButton.Request()
    req.button_id = int(button_id)
    res = _call(ReadButton, "/tf_button/read", req)
    return bool(res.switched_on)


def wait_for_button_press(
    button_id: int, red: int, green: int, blue: int, timeout_sec: float = 0.0
) -> bool:
    req = WaitForButton.Request()
    req.button_id = int(button_id)
    req.red = int(red)
    req.green = int(green)
    req.blue = int(blue)
    req.switch_mode = False
    req.timeout_sec = float(timeout_sec)
    res = _call(
        WaitForButton,
        "/tf_button/wait",
        req,
        timeout_sec=max(10.0, float(timeout_sec) + 1.0),
    )
    return bool(res.pressed)


def wait_for_button_switch(
    button_id: int, red: int, green: int, blue: int, timeout_sec: float = 0.0
) -> bool:
    req = WaitForButton.Request()
    req.button_id = int(button_id)
    req.red = int(red)
    req.green = int(green)
    req.blue = int(blue)
    req.switch_mode = True
    req.timeout_sec = float(timeout_sec)
    res = _call(
        WaitForButton,
        "/tf_button/wait",
        req,
        timeout_sec=max(10.0, float(timeout_sec) + 1.0),
    )
    return bool(res.switched_on)
