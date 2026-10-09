"""Button UIDs come from the connected-bricklet list, not from fixed slots (PR-1962).

The HTTP layer is stubbed. Nothing here opens a socket or talks to brickd.
"""

from __future__ import annotations

import importlib.util
import logging
import sys
import types
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import pytest
from tinkerforge.ip_connection import Error as TinkerforgeError

REPO_ROOT = Path(__file__).resolve().parents[2]
BUTTON_DIR = REPO_ROOT / "ros_packages" / "button_service"
MOTORS_DIR = REPO_ROOT / "ros_packages" / "motors" / "pib_motors"
BRICKLET_PATH = MOTORS_DIR / "pib_motors" / "bricklet.py"
NODE_PATH = BUTTON_DIR / "button_service_node_pkg" / "button_service_node.py"
BLOCKLY_PATH = BUTTON_DIR / "button_service_node_pkg" / "blockly_client.py"

for path in (str(BUTTON_DIR), str(MOTORS_DIR), str(REPO_ROOT / "pib_api" / "client")):
    if path not in sys.path:
        sys.path.insert(0, path)

from pib_api_client import bricklet_client  # noqa: E402

RGB_NAME = "RGB LED Button Bricklet"
SERVO_NAME = "Servo Bricklet 2.0"
RELAY_NAME = "Solid State Relay Bricklet 2.0"
RGB_IDENTIFIER = 282
SERVO_IDENTIFIER = 2157
RELAY_IDENTIFIER = 296

# Measured on the robot, then shuffled so a test cannot pass by keeping list order.
SHUFFLED_ROBOT = [
    {
        "deviceIdentifier": RGB_IDENTIFIER,
        "name": RGB_NAME,
        "parentUid": "29yN",
        "port": "h",
        "uid": "24eF",
    },
    {
        "deviceIdentifier": SERVO_IDENTIFIER,
        "name": SERVO_NAME,
        "parentUid": "29yN",
        "port": "c",
        "uid": "SF1",
    },
    {
        "deviceIdentifier": RELAY_IDENTIFIER,
        "name": RELAY_NAME,
        "parentUid": "29yN",
        "port": "e",
        "uid": "27Fn",
    },
    {
        "deviceIdentifier": RGB_IDENTIFIER,
        "name": RGB_NAME,
        "parentUid": "29yN",
        "port": "f",
        "uid": "24eD",
    },
    {
        "deviceIdentifier": SERVO_IDENTIFIER,
        "name": SERVO_NAME,
        "parentUid": "29yN",
        "port": "a",
        "uid": "2iJK",
    },
    {
        "deviceIdentifier": RGB_IDENTIFIER,
        "name": RGB_NAME,
        "parentUid": "29yN",
        "port": "g",
        "uid": "2dcw",
    },
    {
        "deviceIdentifier": SERVO_IDENTIFIER,
        "name": SERVO_NAME,
        "parentUid": "29yN",
        "port": "d",
        "uid": "29FA",
    },
    {
        "deviceIdentifier": SERVO_IDENTIFIER,
        "name": SERVO_NAME,
        "parentUid": "29yN",
        "port": "b",
        "uid": "SHT",
    },
]


def _rgb(
    port: str, uid: str, name: str = RGB_NAME, identifier: int = RGB_IDENTIFIER
) -> dict:
    return {
        "deviceIdentifier": identifier,
        "name": name,
        "parentUid": "29yN",
        "port": port,
        "uid": uid,
    }


class _Logger:
    def __init__(self) -> None:
        self.records: list[tuple[str, str]] = []

    def info(self, message: str) -> None:
        self.records.append(("INFO", message))

    def warn(self, message: str) -> None:
        self.records.append(("WARN", message))

    def warning(self, message: str) -> None:
        self.records.append(("WARN", message))

    def error(self, message: str) -> None:
        self.records.append(("ERROR", message))


class _Node:
    def __init__(self, name: str) -> None:
        self.name = name
        self.logger = _Logger()
        self.services: list[tuple[str, object]] = []

    def get_logger(self) -> _Logger:
        return self.logger

    def create_service(self, _srv_type, name: str, callback) -> None:
        self.services.append((name, callback))

    def create_publisher(self, *_args, **_kwargs):
        return None

    def destroy_node(self) -> None:
        pass


class _FakeIPConnection:
    def connect(self, _host, _port) -> None:
        return None

    def disconnect(self) -> None:
        return None


class _FakeButton:
    """Stand-in for BrickletRGBLEDButton. One UID can reject itself as the wrong device."""

    BUTTON_STATE_PRESSED = 1
    CALLBACK_BUTTON_STATE_CHANGED = 42
    raise_for: dict[str, Exception] = {}

    def __init__(self, uid: str, ipcon) -> None:
        self.uid = uid
        self.ipcon = ipcon
        self.color = None

    def get_button_state(self) -> int:
        error = type(self).raise_for.get(self.uid)
        if error is not None:
            raise error
        return 0

    def register_callback(self, _callback_id, _callback) -> None:
        return None

    def set_color(self, red: int, green: int, blue: int) -> None:
        self.color = (red, green, blue)


def _ros_modules() -> dict[str, types.ModuleType]:
    rclpy = types.ModuleType("rclpy")
    rclpy.ok = lambda: True
    rclpy.init = lambda args=None: None
    rclpy.spin_once = lambda *_args, **_kwargs: None
    rclpy.spin_until_future_complete = lambda *_args, **_kwargs: None
    rclpy.create_node = lambda name: _Node(name)

    node_mod = types.ModuleType("rclpy.node")
    node_mod.Node = _Node

    button_service = types.ModuleType("button_service")
    srv = types.ModuleType("button_service.srv")
    for name in (
        "ReadButton",
        "SetButtonColor",
        "WaitForButton",
        "SetButtonManualOverride",
    ):
        setattr(srv, name, type(name, (), {}))
    button_service.srv = srv

    datatypes = types.ModuleType("datatypes")
    msg = types.ModuleType("datatypes.msg")

    class ButtonColor:
        def __init__(self) -> None:
            self.bricklet_uid = ""
            self.red = 0
            self.green = 0
            self.blue = 0
            self.sticky = False
            self.clear = False

    msg.ButtonColor = ButtonColor
    datatypes.msg = msg
    return {
        "rclpy": rclpy,
        "rclpy.node": node_mod,
        "button_service": button_service,
        "button_service.srv": srv,
        "datatypes": datatypes,
        "datatypes.msg": msg,
    }


def _load_module(module_name: str, path: Path):
    spec = importlib.util.spec_from_file_location(module_name, path)
    module = importlib.util.module_from_spec(spec)
    with mock.patch.dict(sys.modules, _ros_modules()):
        spec.loader.exec_module(module)
    return module


def _button_node():
    return _load_module("button_service_node_under_test", NODE_PATH)


def _blockly():
    return _load_module("blockly_client_under_test", BLOCKLY_PATH)


def _service(monkeypatch, devices, raise_for=None):
    monkeypatch.delenv("TF_BUTTON_BRICKLET_NUMBERS", raising=False)
    monkeypatch.delenv("TF_BUTTON_UIDS", raising=False)
    module = _button_node()
    _FakeButton.raise_for = dict(raise_for or {})
    monkeypatch.setattr(
        bricklet_client,
        "get_connected_bricklets",
        lambda: (True, {"bricklets": devices}),
        raising=False,
    )
    monkeypatch.setattr(module, "IPConnection", _FakeIPConnection)
    monkeypatch.setattr(module, "BrickletRGBLEDButton", _FakeButton)
    monkeypatch.setattr(module.time, "sleep", lambda *_args, **_kwargs: None)
    return module.TinkerforgeButtonService()


def _color_response(node, button_id: int):
    response = SimpleNamespace(success=True, message="")
    request = SimpleNamespace(button_id=button_id, red=1, green=2, blue=3)
    node.handle_set_color(request, response)
    return response


def test_rgb_buttons_resolve_by_type_and_port_order():
    from button_service_node_pkg.button_resolution import resolve_buttons

    resolved = resolve_buttons(SHUFFLED_ROBOT)

    assert [resolved[button_id].uid for button_id in (1, 2, 3)] == [
        "24eD",
        "2dcw",
        "24eF",
    ]
    assert [resolved[button_id].port for button_id in (1, 2, 3)] == ["f", "g", "h"]
    assert [resolved[button_id].reason for button_id in (1, 2, 3)] == [None, None, None]
    assert "27Fn" not in {slot.uid for slot in resolved.values()}


def test_name_match_wins_over_the_device_identifier():
    """A relay that merely carries the button identifier is not a button."""
    from button_service_node_pkg.button_resolution import (
        RGB_LED_BUTTON_DEVICE_IDENTIFIER,
        resolve_buttons,
    )

    resolved = resolve_buttons(
        [
            {
                "deviceIdentifier": RGB_LED_BUTTON_DEVICE_IDENTIFIER,
                "name": RELAY_NAME,
                "port": "a",
                "uid": "27Fn",
            },
            _rgb("b", "24eD"),
        ]
    )

    assert RGB_LED_BUTTON_DEVICE_IDENTIFIER == 282
    assert resolved[1].uid == "24eD"
    assert resolved[2].uid is None


def test_unnamed_device_falls_back_to_the_button_identifier():
    from button_service_node_pkg.button_resolution import resolve_buttons

    resolved = resolve_buttons(
        [
            {
                "deviceIdentifier": RELAY_IDENTIFIER,
                "name": "",
                "port": "a",
                "uid": "27Fn",
            },
            {
                "deviceIdentifier": RGB_IDENTIFIER,
                "name": "",
                "port": "c",
                "uid": "24eD",
            },
        ]
    )

    assert resolved[1].uid == "24eD"
    assert resolved[1].port == "c"


def test_empty_uid_occupies_its_port_slot_and_names_the_reason():
    from button_service_node_pkg.button_resolution import resolve_buttons

    resolved = resolve_buttons([_rgb("f", ""), _rgb("h", "24eF"), _rgb("g", "2dcw")])

    assert resolved[1].uid is None
    assert resolved[1].port == "f"
    assert "no UID" in resolved[1].reason
    assert resolved[2].uid == "2dcw"
    assert resolved[3].uid == "24eF"


def test_node_maps_ports_and_still_publishes_every_service(monkeypatch):
    node = _service(monkeypatch, SHUFFLED_ROBOT)

    assert [node.buttons[button_id].uid for button_id in (1, 2, 3)] == [
        "24eD",
        "2dcw",
        "24eF",
    ]
    assert {name for name, _callback in node.services} == {
        "/tf_button/set_color",
        "/tf_button/read",
        "/tf_button/wait",
    }


def test_empty_slot_is_logged_once_and_the_other_buttons_stay_up(monkeypatch):
    node = _service(
        monkeypatch,
        [_rgb("f", "  "), _rgb("g", "2dcw"), _rgb("h", "24eF")],
    )

    assert 1 not in node.buttons
    assert node.buttons[2].uid == "2dcw"
    assert node.buttons[3].uid == "24eF"
    empty_logs = [
        message
        for level, message in node.logger.records
        if level == "ERROR" and "no UID" in message
    ]
    assert len(empty_logs) == 1
    assert "f" in empty_logs[0]


def test_wrong_device_type_is_isolated_to_that_button(monkeypatch):
    wrong_type = TinkerforgeError(
        TinkerforgeError.WRONG_DEVICE_TYPE,
        "UID 2dcw belongs to a Solid State Relay Bricklet 2.0 "
        "instead of the expected RGB LED Button Bricklet",
    )
    node = _service(monkeypatch, SHUFFLED_ROBOT, raise_for={"2dcw": wrong_type})

    assert node.buttons[1].uid == "24eD"
    assert 2 not in node.buttons
    assert node.buttons[3].uid == "24eF"
    assert {name for name, _callback in node.services} >= {
        "/tf_button/set_color",
        "/tf_button/read",
        "/tf_button/wait",
    }
    reasons = [
        message
        for level, message in node.logger.records
        if level == "ERROR" and "2dcw" in message
    ]
    assert len(reasons) == 1
    assert "Solid State Relay Bricklet 2.0" in reasons[0]

    kept = _color_response(node, 1)
    assert kept.success is True
    rejected = _color_response(node, 2)
    assert rejected.success is False
    assert "Solid State Relay" in rejected.message


def test_button_id_without_a_device_returns_a_clear_error(monkeypatch):
    node = _service(monkeypatch, [_rgb("f", "24eD"), _rgb("g", "2dcw")])

    missing = _color_response(node, 3)
    assert missing.success is False
    assert "Button 3" in missing.message
    assert "unavailable" in missing.message
    assert "no RGB LED Button" in missing.message

    unknown = _color_response(node, 4)
    assert unknown.success is False
    assert unknown.message == "Unknown button_id 4. Valid ids are 1, 2, 3."


def test_blockly_uses_port_order_instead_of_fixed_slot_numbers(monkeypatch):
    monkeypatch.setenv("TF_BUTTON_BRICKLET_NUMBERS", "5,6,7")
    module = _blockly()

    def fixed_slot_lookup(url, timeout=5):
        number = str(url).rstrip("/").split("/")[-1]
        # The old contract: slot 5 is the relay, 6 and 7 the first two buttons.
        uids = {"5": "27Fn", "6": "24eD", "7": "2dcw"}

        class Response:
            def raise_for_status(self) -> None:
                return None

            def json(self):
                return {"uid": uids[number]}

        return Response()

    if getattr(module, "requests", None) is not None:
        monkeypatch.setattr(module.requests, "get", fixed_slot_lookup)
    monkeypatch.setattr(
        bricklet_client,
        "get_connected_bricklets",
        lambda: (True, {"bricklets": SHUFFLED_ROBOT}),
        raising=False,
    )

    assert module._button_id_to_uid(1) == "24eD"
    assert module._button_id_to_uid(2) == "2dcw"
    assert module._button_id_to_uid(3) == "24eF"


def test_blockly_reports_a_button_id_without_a_device(monkeypatch):
    monkeypatch.delenv("TF_BUTTON_BRICKLET_NUMBERS", raising=False)
    monkeypatch.delenv("TF_BUTTON_UIDS", raising=False)
    module = _blockly()
    monkeypatch.setattr(
        bricklet_client,
        "get_connected_bricklets",
        lambda: (True, {"bricklets": [_rgb("f", "24eD"), _rgb("g", "2dcw")]}),
        raising=False,
    )

    with pytest.raises(RuntimeError, match="Button 3 is unavailable"):
        module._button_id_to_uid(3)


def test_get_connected_bricklets_reads_the_connected_endpoint(monkeypatch):
    captured = {}

    def send_request(request):
        captured["url"] = request.full_url
        captured["method"] = request.get_method()
        return True, {"bricklets": []}

    monkeypatch.setattr(bricklet_client, "send_request", send_request)

    ok, payload = bricklet_client.get_connected_bricklets()

    assert ok is True
    assert payload == {"bricklets": []}
    assert captured["method"] == "GET"
    assert captured["url"].rstrip("/").endswith("/bricklet/connected")


def test_fixed_button_slot_numbers_are_removed_from_config_and_docs():
    paths = [
        REPO_ROOT / "docker-compose.yaml",
        REPO_ROOT / "docs" / "test-basis" / "ros2_interfaces.md",
        REPO_ROOT / "docs" / "test-basis" / "infrastructure_and_deployment.md",
        NODE_PATH,
        BLOCKLY_PATH,
    ]
    offenders = [
        str(path.relative_to(REPO_ROOT))
        for path in paths
        if "TF_BUTTON_BRICKLET_NUMBERS" in path.read_text(encoding="utf-8")
    ]
    assert offenders == []


class _FakeMotorBricklet:
    DEVICE_IDENTIFIER = 2157
    FUNCTION_SET_STATE = 3

    def __init__(self, uid, ipcon):
        self.uid = uid
        self.ipcon = ipcon

    def set_response_expected(self, _function_id, _response_expected) -> None:
        return None


def _fake_tinkerforge_modules() -> dict[str, types.ModuleType]:
    class FakeIPConnection:
        ENUMERATION_TYPE_AVAILABLE = 0
        ENUMERATION_TYPE_DISCONNECTED = 2

        def connect(self, _host, _port) -> None:
            return None

    class FakeError(Exception):
        def __init__(self, value=0, description=""):
            super().__init__(description)
            self.value = value

    def module(name, **attributes):
        fake = types.ModuleType(name)
        for key, value in attributes.items():
            setattr(fake, key, value)
        return fake

    return {
        "tinkerforge": module("tinkerforge"),
        "tinkerforge.brick_hat": module(
            "tinkerforge.brick_hat", BrickHAT=_FakeMotorBricklet
        ),
        "tinkerforge.bricklet_servo_v2": module(
            "tinkerforge.bricklet_servo_v2", BrickletServoV2=_FakeMotorBricklet
        ),
        "tinkerforge.bricklet_solid_state_relay_v2": module(
            "tinkerforge.bricklet_solid_state_relay_v2",
            BrickletSolidStateRelayV2=_FakeMotorBricklet,
        ),
        "tinkerforge.bricklet_rgb_led_button": module(
            "tinkerforge.bricklet_rgb_led_button",
            BrickletRGBLEDButton=_FakeMotorBricklet,
        ),
        "tinkerforge.ip_connection": module(
            "tinkerforge.ip_connection", IPConnection=FakeIPConnection, Error=FakeError
        ),
    }


def _import_bricklet(configured: dict, connected: list[dict]):
    spec = importlib.util.spec_from_file_location(
        "bricklet_mismatch_under_test", BRICKLET_PATH
    )
    module = importlib.util.module_from_spec(spec)
    with (
        mock.patch.dict(sys.modules, _fake_tinkerforge_modules()),
        mock.patch.object(
            bricklet_client, "get_all_bricklets", return_value=(True, configured)
        ),
        mock.patch.object(
            bricklet_client,
            "get_connected_bricklets",
            return_value=(True, {"bricklets": connected}),
            create=True,
        ),
    ):
        spec.loader.exec_module(module)
    return module


def test_servo_configured_with_a_relay_uid_is_logged_and_skipped(caplog):
    configured = {
        "bricklets": [
            {"type": "Servo Bricklet", "uid": "2iJK"},
            {"type": "Servo Bricklet", "uid": "SHT"},
            {"type": "Solid State Relay Bricklet", "uid": "27Fn"},
            {"type": "RGB LED Button Bricklet", "uid": "24eD"},
        ]
    }
    connected = [
        {
            "uid": "2iJK",
            "name": RELAY_NAME,
            "deviceIdentifier": RELAY_IDENTIFIER,
            "port": "a",
        },
        {
            "uid": "SHT",
            "name": SERVO_NAME,
            "deviceIdentifier": SERVO_IDENTIFIER,
            "port": "b",
        },
        {
            "uid": "27Fn",
            "name": RELAY_NAME,
            "deviceIdentifier": RELAY_IDENTIFIER,
            "port": "e",
        },
        {
            "uid": "24eD",
            "name": RGB_NAME,
            "deviceIdentifier": RGB_IDENTIFIER,
            "port": "f",
        },
    ]
    caplog.set_level(logging.ERROR)

    module = _import_bricklet(configured, connected)

    assert list(module.uid_to_servo_bricklet) == ["SHT"]
    assert module.solid_state_relay_bricklet.uid == "27Fn"
    assert list(module.uid_to_rgb_led_bricklet) == ["24eD"]
    mismatches = [
        record.getMessage()
        for record in caplog.records
        if record.levelno >= logging.ERROR and "2iJK" in record.getMessage()
    ]
    assert len(mismatches) == 1
    assert (
        "configured as Servo Bricklet, detected as Solid State Relay Bricklet 2.0"
        in mismatches[0]
    )
