"""Unit tests for the Bricklet UID selection of the motor nodes (PR-1796).

Covers both halves of the fix: the dependency-free rule in `bricklet_mapping`
and the guarantee that importing `bricklet.py` survives a UID that tinkerforge
would reject.
"""

import ast
import importlib.util
import os
import sys
import types
import unittest
from pathlib import Path
from unittest import mock

REPO_ROOT = Path(__file__).resolve().parents[2]
MOTORS_DIR = REPO_ROOT / "ros_packages" / "motors" / "pib_motors"
BRICKLET_MODULE_PATH = MOTORS_DIR / "pib_motors" / "bricklet.py"

for path in (str(REPO_ROOT / "pib_api" / "client"), str(MOTORS_DIR)):
    if path not in sys.path:
        sys.path.insert(0, path)

from pib_api_client import bricklet_client  # noqa: E402
from pib_motors import bricklet_mapping  # noqa: E402

# 'E2E001' contains a '0' and is the UID that took the motor stack down.
BRICKLET_DTOS = {
    "bricklets": [
        {"type": "Servo Bricklet", "uid": "E2E001"},
        {"type": "Servo Bricklet", "uid": "Servo2"},
        {"type": "Servo Bricklet", "uid": ""},
        {"type": "Solid State Relay Bricklet", "uid": "RELAY1"},
        {"type": "RGB LED Button Bricklet", "uid": "Button"},
    ]
}


class TestBrickletMapping(unittest.TestCase):
    def test_module_stays_free_of_hardware_dependencies(self):
        tree = ast.parse(
            (MOTORS_DIR / "pib_motors" / "bricklet_mapping.py").read_text()
        )
        imported = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported.update(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom):
                imported.add(node.module or "")

        self.assertEqual(
            [
                name
                for name in imported
                if name.split(".")[0] in ("tinkerforge", "requests")
            ],
            [],
        )

    def test_keeps_every_valid_uid_in_order(self):
        uids = ["A", "SRV123", "Servo1", "abcdef", "zzzzzz", "9"]

        selection = bricklet_mapping.select_bricklet_uids(uids)

        self.assertEqual(selection.valid, uids)
        self.assertEqual(selection.skipped, [])

    def test_skips_an_invalid_uid_and_reports_it(self):
        selection = bricklet_mapping.select_bricklet_uids(
            ["Servo1", "E2E001", "Servo2"]
        )

        self.assertEqual(selection.valid, ["Servo1", "Servo2"])
        self.assertEqual([entry.uid for entry in selection.skipped], ["E2E001"])
        self.assertIn("invalid format", selection.skipped[0].reason)
        for character in bricklet_mapping.UID_FORBIDDEN_CHARACTERS:
            self.assertIn(f"'{character}'", selection.skipped[0].reason)

    def test_rejects_every_non_base58_uid(self):
        invalid = ["E2E001", "SERVO1", "ABCI12", "abcl12", "SERVO12", "bad uid!"]

        selection = bricklet_mapping.select_bricklet_uids(invalid)

        self.assertEqual(selection.valid, [])
        self.assertEqual([entry.uid for entry in selection.skipped], invalid)

    def test_unconfigured_uids_are_dropped_without_a_complaint(self):
        selection = bricklet_mapping.select_bricklet_uids(["", "   ", None])

        self.assertEqual(selection.valid, [])
        self.assertEqual(selection.skipped, [])

    def test_a_non_string_uid_is_reported_rather_than_raised(self):
        selection = bricklet_mapping.select_bricklet_uids([42])

        self.assertEqual(selection.valid, [])
        self.assertEqual(selection.skipped[0].uid, "42")
        self.assertIn("expected a string", selection.skipped[0].reason)


class _FakeBricklet:
    """A tinkerforge bricklet that refuses non-Base58 UIDs, like the real one."""

    DEVICE_IDENTIFIER = 2157
    FUNCTION_SET_STATE = 3
    unusable_uids: set[str] = set()

    def __init__(self, uid, ipcon):
        if any(character in uid for character in "0OIl"):
            raise _FakeError(-13, f'UID "{uid}" contains invalid character')
        if uid in _FakeBricklet.unusable_uids:
            raise _FakeError(-6, f'UID "{uid}" is not usable')
        self.uid = uid
        self.ipcon = ipcon

    def set_response_expected(self, function_id, response_expected):
        pass


class _FakeError(Exception):
    TIMEOUT = -1
    WRONG_RESPONSE_LENGTH = -3
    INVALID_UID = -6
    NOT_CONNECTED = -8

    def __init__(self, value=0, description=""):
        super().__init__(description)
        self.value = value


def _fake_tinkerforge_modules() -> dict[str, types.ModuleType]:
    class FakeIPConnection:
        ENUMERATION_TYPE_AVAILABLE = 0
        ENUMERATION_TYPE_DISCONNECTED = 2

        def connect(self, host, port):
            pass

    def module(name, **attributes):
        fake = types.ModuleType(name)
        for key, value in attributes.items():
            setattr(fake, key, value)
        return fake

    return {
        "tinkerforge": module("tinkerforge"),
        "tinkerforge.brick_hat": module(
            "tinkerforge.brick_hat", BrickHAT=_FakeBricklet
        ),
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
            Error=_FakeError,
        ),
    }


def _import_bricklet_module(bricklet_dtos):
    """Import bricklet.py the way the motor_control node does, in isolation."""
    spec = importlib.util.spec_from_file_location(
        "bricklet_under_test", BRICKLET_MODULE_PATH
    )
    module = importlib.util.module_from_spec(spec)

    with (
        mock.patch.dict(sys.modules, _fake_tinkerforge_modules()),
        mock.patch.object(
            bricklet_client,
            "get_all_bricklets",
            mock.Mock(return_value=(True, bricklet_dtos)),
        ),
    ):
        spec.loader.exec_module(module)

    return module


class TestBrickletImportSurvivesABadUid(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ.setdefault("FLASK_API_BASE_URL", "http://localhost:5000")

    def tearDown(self):
        _FakeBricklet.unusable_uids = set()

    def test_import_succeeds_and_keeps_the_remaining_bricklets(self):
        with self.assertLogs(level="ERROR"):
            module = _import_bricklet_module(BRICKLET_DTOS)

        self.assertEqual(list(module.uid_to_servo_bricklet), ["Servo2"])
        self.assertEqual(list(module.uid_to_rgb_led_bricklet), ["Button"])
        self.assertEqual(module.solid_state_relay_bricklet.uid, "RELAY1")

    def test_the_skipped_uid_and_the_reason_are_logged_at_error_level(self):
        with self.assertLogs(level="ERROR") as logs:
            _import_bricklet_module(BRICKLET_DTOS)

        errors = [
            record.getMessage()
            for record in logs.records
            if record.levelname == "ERROR"
        ]
        self.assertEqual(len(errors), 1)
        self.assertIn("E2E001", errors[0])
        self.assertIn("Servo Bricklet", errors[0])
        self.assertIn("invalid format", errors[0])

    def test_an_all_valid_configuration_logs_nothing_and_skips_nothing(self):
        dtos = {
            "bricklets": [
                {"type": "Servo Bricklet", "uid": "Servo1"},
                {"type": "Solid State Relay Bricklet", "uid": "RELAY1"},
                {"type": "RGB LED Button Bricklet", "uid": "Button"},
            ]
        }

        with mock.patch("logging.error") as log_error:
            module = _import_bricklet_module(dtos)

        log_error.assert_not_called()
        self.assertEqual(list(module.uid_to_servo_bricklet), ["Servo1"])

    def test_a_valid_uid_rejected_by_tinkerforge_is_logged_and_skipped(self):
        _FakeBricklet.unusable_uids = {"Servo2"}

        with self.assertLogs(level="ERROR") as logs:
            module = _import_bricklet_module(BRICKLET_DTOS)

        self.assertEqual(list(module.uid_to_servo_bricklet), [])
        self.assertTrue(
            any(
                "Servo2" in record.getMessage() and "tinkerforge" in record.getMessage()
                for record in logs.records
            )
        )


if __name__ == "__main__":
    unittest.main()
