"""Unit tests for the pib-api bricklet load retry on startup (PR-1509)."""

import importlib.util
import os
import sys
import types
import unittest
from pathlib import Path
from unittest import mock
from urllib.error import URLError

REPO_ROOT = Path(__file__).resolve().parents[2]
BRICKLET_MODULE_PATH = (
    REPO_ROOT / "ros_packages" / "motors" / "pib_motors" / "pib_motors" / "bricklet.py"
)

for path in (
    str(REPO_ROOT / "pib_api" / "client"),
    str(REPO_ROOT / "ros_packages" / "motors" / "pib_motors"),
):
    if path not in sys.path:
        sys.path.insert(0, path)

# Imported here so that the module the retry loop calls into stays the same object
# across the sys.modules patching done while importing bricklet.py.
from pib_api_client import bricklet_client  # noqa: E402

BRICKLET_DTOS = {
    "bricklets": [
        {"type": "Servo Bricklet", "uid": "SERVO1"},
        {"type": "Solid State Relay Bricklet", "uid": "RELAY1"},
        {"type": "RGB LED Button Bricklet", "uid": "BUTTON1"},
    ]
}

CONNECTION_REFUSED = URLError(OSError(111, "Connection refused"))


class _FakeBricklet:
    DEVICE_IDENTIFIER = 2157
    FUNCTION_SET_STATE = 3

    def __init__(self, uid, ipcon):
        self.uid = uid
        self.ipcon = ipcon

    def set_response_expected(self, function_id, response_expected):
        pass


def _fake_tinkerforge_modules() -> dict[str, types.ModuleType]:
    """Stand in for the tinkerforge stack, which needs a running brickd."""

    class FakeIPConnection:
        ENUMERATION_TYPE_AVAILABLE = 0
        ENUMERATION_TYPE_DISCONNECTED = 2

        def connect(self, host, port):
            pass

    class FakeError(Exception):
        TIMEOUT = -1
        WRONG_RESPONSE_LENGTH = -3
        INVALID_UID = -6
        NOT_CONNECTED = -8

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
        "tinkerforge.brick_hat": module("tinkerforge.brick_hat", BrickHAT=_FakeBricklet),
        "tinkerforge.bricklet_servo_v2": module(
            "tinkerforge.bricklet_servo_v2", BrickletServoV2=_FakeBricklet
        ),
        "tinkerforge.bricklet_solid_state_relay_v2": module(
            "tinkerforge.bricklet_solid_state_relay_v2",
            BrickletSolidStateRelayV2=_FakeBricklet,
        ),
        "tinkerforge.bricklet_rgb_led_button": module(
            "tinkerforge.bricklet_rgb_led_button", BrickletRGBLEDButton=_FakeBricklet
        ),
        "tinkerforge.ip_connection": module(
            "tinkerforge.ip_connection",
            IPConnection=FakeIPConnection,
            Error=FakeError,
        ),
    }


def _import_bricklet_module(get_all_bricklets, sleep):
    """Import bricklet.py the way the motor_control node does, in isolation."""
    spec = importlib.util.spec_from_file_location(
        "bricklet_under_test", BRICKLET_MODULE_PATH
    )
    module = importlib.util.module_from_spec(spec)

    with mock.patch.dict(sys.modules, _fake_tinkerforge_modules()), mock.patch.object(
        bricklet_client, "get_all_bricklets", get_all_bricklets
    ), mock.patch("time.sleep", sleep):
        spec.loader.exec_module(module)

    return module


class TestBrickletLoadRetry(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ.setdefault("FLASK_API_BASE_URL", "http://localhost:5000")

    def test_module_imports_once_pib_api_becomes_reachable(self):
        get_all_bricklets = mock.Mock(
            side_effect=[(False, None), (False, None), (True, BRICKLET_DTOS)]
        )
        sleep = mock.Mock()

        module = _import_bricklet_module(get_all_bricklets, sleep)

        self.assertEqual(module.bricklet_dtos, BRICKLET_DTOS)
        self.assertEqual(get_all_bricklets.call_count, 3)
        self.assertEqual(sleep.call_count, 2)
        self.assertEqual(sleep.call_args_list, [mock.call(2.0), mock.call(2.0)])

    def test_bricklets_are_wired_up_after_a_retry(self):
        get_all_bricklets = mock.Mock(side_effect=[(False, None), (True, BRICKLET_DTOS)])

        module = _import_bricklet_module(get_all_bricklets, mock.Mock())

        self.assertEqual(list(module.uid_to_servo_bricklet), ["SERVO1"])
        self.assertEqual(list(module.uid_to_rgb_led_bricklet), ["BUTTON1"])
        self.assertEqual(module.solid_state_relay_bricklet.uid, "RELAY1")

    def test_retries_when_the_request_raises_connection_refused(self):
        module = _import_bricklet_module(
            mock.Mock(return_value=(True, BRICKLET_DTOS)), mock.Mock()
        )
        get_all_bricklets = mock.Mock(
            side_effect=[CONNECTION_REFUSED, CONNECTION_REFUSED, (True, BRICKLET_DTOS)]
        )
        sleep = mock.Mock()

        with mock.patch.object(
            bricklet_client, "get_all_bricklets", get_all_bricklets
        ), mock.patch("time.sleep", sleep):
            dtos = module.load_bricklets()

        self.assertEqual(dtos, BRICKLET_DTOS)
        self.assertEqual(get_all_bricklets.call_count, 3)
        self.assertEqual(sleep.call_count, 2)

    def test_each_retry_logs_a_warning(self):
        module = _import_bricklet_module(
            mock.Mock(return_value=(True, BRICKLET_DTOS)), mock.Mock()
        )
        get_all_bricklets = mock.Mock(side_effect=[(False, None), (True, BRICKLET_DTOS)])

        with mock.patch.object(
            bricklet_client, "get_all_bricklets", get_all_bricklets
        ), mock.patch("time.sleep"), self.assertLogs(level="WARNING") as logs:
            module.load_bricklets()

        self.assertEqual(len(logs.records), 1)
        self.assertIn("attempt 1/30", logs.output[0])
        self.assertIn("retrying in 2.0s", logs.output[0])

    def test_gives_up_with_runtime_error_after_the_retry_window(self):
        get_all_bricklets = mock.Mock(return_value=(False, None))
        sleep = mock.Mock()

        with self.assertRaises(RuntimeError) as raised:
            _import_bricklet_module(get_all_bricklets, sleep)

        self.assertIn("failed to load bricklets from pib-api", str(raised.exception))
        self.assertEqual(get_all_bricklets.call_count, 30)
        self.assertEqual(sleep.call_count, 29)

    def test_retry_window_spans_at_least_a_minute(self):
        module = _import_bricklet_module(
            mock.Mock(return_value=(True, BRICKLET_DTOS)), mock.Mock()
        )

        self.assertEqual(module.BRICKLET_LOAD_TIMEOUT_SECONDS, 60.0)
        self.assertEqual(module.BRICKLET_LOAD_RETRY_INTERVAL_SECONDS, 2.0)


if __name__ == "__main__":
    unittest.main()
