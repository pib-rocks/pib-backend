"""PC-125: unit tests for FeetechStSerialActuator and serial adapter resolution."""

from __future__ import annotations

import importlib.util
import sys
import types
from pathlib import Path
from unittest import mock

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
PIB_MOTORS_DIR = REPO_ROOT / "ros_packages" / "motors" / "pib_motors"
ACTUATOR_MODULE_PATH = PIB_MOTORS_DIR / "pib_motors" / "actuator.py"
FEETECH_MODULE_PATH = PIB_MOTORS_DIR / "pib_motors" / "feetech_actuator.py"

for path in (
    str(REPO_ROOT / "pib_api" / "client"),
    str(PIB_MOTORS_DIR),
):
    if path not in sys.path:
        sys.path.insert(0, path)


class FakeStsPacketHandler:
    """Fakes pib_motors.stservo_sdk.sts.sts without opening a physical serial port."""

    def __init__(self, ph) -> None:
        self.ph = ph
        self.pos_ticks: int = 2048
        self.speed: int = 0
        self.current: int = 15
        self.temperature: int = 34
        self.comm_result: int = 0  # COMM_SUCCESS
        self.writes: list[tuple[int, int, int, int]] = []
        self.byte_writes: list[tuple[int, int, int]] = []

    def ReadPosSpeed(self, servo_id: int):
        return self.pos_ticks, self.speed, self.comm_result, 0

    def ReadCurrent(self, servo_id: int):
        return self.current, self.comm_result, 0

    def WritePosEx(self, servo_id: int, position: int, speed: int, acc: int):
        self.writes.append((servo_id, position, speed, acc))
        if self.comm_result == 0:
            self.pos_ticks = position
        return self.comm_result, 0

    def write1ByteTxRx(self, servo_id: int, address: int, data: int):
        self.byte_writes.append((servo_id, address, data))
        return self.comm_result, 0

    def read1ByteTxRx(self, servo_id: int, address: int):
        if address == 62:
            return getattr(self, "voltage_raw", 118), self.comm_result, 0
        return self.temperature, self.comm_result, 0


class FakePortHandler:
    """Fakes PortHandler without pyserial opening hardware."""

    def __init__(self, port_name: str, can_open: bool = True) -> None:
        self.port_name = port_name
        self.can_open = can_open
        self.is_open = False
        self.baudrate = 1_000_000

    def openPort(self) -> bool:
        if not self.can_open:
            return False
        self.is_open = True
        return True

    def setBaudRate(self, baudrate: int) -> bool:
        self.baudrate = baudrate
        return self.can_open

    def closePort(self) -> None:
        self.is_open = False


def _load_modules():
    """Load feetech_actuator and actuator with fake tinkerforge dependencies."""
    fake_tf = types.ModuleType("tinkerforge")
    fake_servo = types.ModuleType("tinkerforge.bricklet_servo_v2")
    fake_servo.BrickletServoV2 = object
    fake_ipcon = types.ModuleType("tinkerforge.ip_connection")
    fake_ipcon.Error = Exception
    fake_bricklet = types.ModuleType("pib_motors.bricklet")
    fake_bricklet.uid_to_servo_bricklet = {}

    with mock.patch.dict(
        sys.modules,
        {
            "tinkerforge": fake_tf,
            "tinkerforge.bricklet_servo_v2": fake_servo,
            "tinkerforge.ip_connection": fake_ipcon,
            "pib_motors.bricklet": fake_bricklet,
        },
    ):
        spec_ft = importlib.util.spec_from_file_location(
            "pib_motors.feetech_actuator", FEETECH_MODULE_PATH
        )
        ft_mod = importlib.util.module_from_spec(spec_ft)
        sys.modules["pib_motors.feetech_actuator"] = ft_mod
        spec_ft.loader.exec_module(ft_mod)

        spec_act = importlib.util.spec_from_file_location(
            "pib_motors.actuator_test", ACTUATOR_MODULE_PATH
        )
        act_mod = importlib.util.module_from_spec(spec_act)
        spec_act.loader.exec_module(act_mod)

    return ft_mod, act_mod


def test_resolve_serial_device_matches_raw_serial_and_direct_paths(tmp_path):
    ft_mod, _ = _load_modules()

    direct_dev = tmp_path / "ttyMotor0"
    direct_dev.write_text("")
    assert ft_mod.resolve_serial_device(str(direct_dev), by_id_dir=tmp_path) == str(
        direct_dev
    )

    by_id = tmp_path / "by-id"
    by_id.mkdir()
    target = tmp_path / "ttyACM0"
    target.write_text("")
    symlink = by_id / "usb-1a86_USB_Single_Serial_5B3D042723-if00"
    symlink.symlink_to(target)

    # Raw Waveshare serial entered in System/Hardware-IDs resolves to the by-id path
    resolved = ft_mod.resolve_serial_device("  5B3D042723  ", by_id_dir=by_id)
    assert resolved == str(symlink)

    # Unmatched identifier returns stripped value so PortHandler reports it clearly
    assert ft_mod.resolve_serial_device("UNKNOWN_ID", by_id_dir=by_id) == "UNKNOWN_ID"


def test_centideg_and_ticks_conversion():
    ft_mod, _ = _load_modules()

    assert ft_mod.centideg_to_ticks(0) == ft_mod.TICKS_AT_JOINT_ZERO
    assert ft_mod.centideg_to_ticks(9000) == 3072
    assert ft_mod.centideg_to_ticks(-9000) == 1024
    assert ft_mod.centideg_to_ticks(4500) == ft_mod.TICKS_AT_JOINT_ZERO + 512
    assert ft_mod.centideg_to_ticks(-4500) == ft_mod.TICKS_AT_JOINT_ZERO - 512
    assert ft_mod.centideg_to_ticks(9000) != ft_mod.centideg_to_ticks(-9000)

    assert ft_mod.ticks_to_centideg(2048) == 0
    assert ft_mod.ticks_to_centideg(1024) == -9000
    assert ft_mod.ticks_to_centideg(3072) == 9000
    assert ft_mod.ticks_to_centideg(0) == -18000


def test_centideg_to_ticks_roundtrip_property():
    ft_mod, _ = _load_modules()
    one_tick_centideg = ft_mod.CENTIDEG_PER_REVOLUTION / ft_mod.TICKS_PER_REVOLUTION

    for centideg in range(-9000, 9001, 13):
        ticks = ft_mod.centideg_to_ticks(centideg)
        back = ft_mod.ticks_to_centideg(ticks)
        assert abs(back - centideg) <= one_tick_centideg


def test_ticks_to_centideg_roundtrip_property():
    ft_mod, _ = _load_modules()

    for ticks in range(0, 4096):
        centideg = ft_mod.ticks_to_centideg(ticks)
        if centideg < -9000 or centideg > 9000:
            continue
        back = ft_mod.centideg_to_ticks(centideg)
        assert abs(back - ticks) <= 1


def test_centideg_to_ticks_rejects_out_of_range():
    ft_mod, _ = _load_modules()

    with pytest.raises(ValueError, match=r"10001.*\[-9000, 9000\]"):
        ft_mod.centideg_to_ticks(10001)

    with pytest.raises(ValueError, match=r"-10001.*\[-9000, 9000\]"):
        ft_mod.centideg_to_ticks(-10001)

    with pytest.raises(ValueError, match=r"5000.*\[-2000, 4500\]"):
        ft_mod.centideg_to_ticks(
            5000, rotation_range_min=-2000, rotation_range_max=4500
        )


def test_create_actuator_builds_feetech_serial_and_registers_capabilities(monkeypatch):
    ft_mod, act_mod = _load_modules()
    fake_pk = None

    def fake_sts(ph):
        nonlocal fake_pk
        fake_pk = FakeStsPacketHandler(ph)
        return fake_pk

    monkeypatch.setattr(ft_mod, "PortHandler", FakePortHandler)
    monkeypatch.setattr(ft_mod, "sts", fake_sts)
    ft_mod._port_cache.clear()

    actuator = act_mod.create_actuator("feetech_st_serial", 51, "5B3D042723", "max")
    assert isinstance(actuator, ft_mod.FeetechStSerialActuator)
    assert isinstance(actuator, act_mod.Actuator)
    assert actuator.is_connected() is True

    cap = act_mod.Capability
    assert act_mod.capabilities_for("feetech_st_serial") == frozenset(
        {
            cap.CURRENT,
            cap.TARGET_POSITION,
            cap.ACTUAL_POSITION,
            cap.TEMPERATURE,
        }
    )


def test_feetech_actuator_uses_sts3095_and_finger_speed_classes_and_inversion(
    monkeypatch,
):
    ft_mod, act_mod = _load_modules()
    handlers: dict[str, FakeStsPacketHandler] = {}

    def fake_sts(ph):
        pk = FakeStsPacketHandler(ph)
        handlers[ph.port_name] = pk
        return pk

    monkeypatch.setattr(ft_mod, "PortHandler", FakePortHandler)
    monkeypatch.setattr(ft_mod, "sts", fake_sts)
    ft_mod._port_cache.clear()

    # Pin 18 is elbow_right (STS3095 heavy servo, inverted)
    elbow = act_mod.create_actuator("feetech_st_serial", 18, "WS-ARM", "min")
    # Pin 11 is thumb_right_stretch (finger servo, non-inverted)
    thumb = act_mod.create_actuator("feetech_st_serial", 11, "WS-ARM", "max")

    # Both share the same cached port handler on 'WS-ARM'
    assert elbow._pk is thumb._pk
    pk = handlers["WS-ARM"]

    assert elbow.set_position(4500) is True
    assert thumb.set_position(4500) is True

    # Inverted +4500 -> -4500 centideg -> 1536 ticks, speed 700, acc 30
    # Non-inverted +4500 -> +4500 centideg -> 2560 ticks, speed 3000, acc 100
    assert pk.writes == [
        (18, 1536, 700, 30),
        (11, 2560, 3000, 100),
    ]
    assert elbow.get_position() == 4500
    assert thumb.get_position() == 4500
    assert thumb.get_current() == 150  # 15 * 10 mA
    assert thumb.get_temperature() == 34


def test_feetech_actuator_get_voltage_reads_present_voltage_register(monkeypatch):
    ft_mod, act_mod = _load_modules()
    pk_instance = None

    def fake_sts(ph):
        nonlocal pk_instance
        pk_instance = FakeStsPacketHandler(ph)
        pk_instance.voltage_raw = 118
        return pk_instance

    monkeypatch.setattr(ft_mod, "PortHandler", FakePortHandler)
    monkeypatch.setattr(ft_mod, "sts", fake_sts)
    ft_mod._port_cache.clear()

    actuator = act_mod.create_actuator("feetech_st_serial", 51, "WS-HEAD", "max")
    assert actuator.get_voltage() == 11.8


def test_feetech_actuator_reconnects_lazily_when_ssr_powers_bus(monkeypatch):
    ft_mod, act_mod = _load_modules()
    pk_instance = None

    def fake_sts(ph):
        nonlocal pk_instance
        pk_instance = FakeStsPacketHandler(ph)
        # Bus initially unpowered (SSR off): ReadPosSpeed fails
        pk_instance.comm_result = -6
        return pk_instance

    monkeypatch.setattr(ft_mod, "PortHandler", FakePortHandler)
    monkeypatch.setattr(ft_mod, "sts", fake_sts)
    ft_mod._port_cache.clear()

    actuator = act_mod.create_actuator("feetech_st_serial", 51, "WS-HEAD", "max")
    assert actuator._connected is False
    assert actuator.set_position(1000) is False

    # SSR turns on -> bus starts replying
    pk_instance.comm_result = 0
    assert actuator.set_position(0) is True
    assert actuator._connected is True
    assert pk_instance.writes == [(51, ft_mod.TICKS_AT_JOINT_ZERO, 700, 30)]


def test_feetech_actuator_applies_torque_enable_setting(monkeypatch):
    ft_mod, act_mod = _load_modules()
    pk_instance = None

    def fake_sts(ph):
        nonlocal pk_instance
        pk_instance = FakeStsPacketHandler(ph)
        return pk_instance

    monkeypatch.setattr(ft_mod, "PortHandler", FakePortHandler)
    monkeypatch.setattr(ft_mod, "sts", fake_sts)
    ft_mod._port_cache.clear()

    actuator = act_mod.create_actuator("feetech_st_serial", 51, "WS-HEAD", "max")
    assert actuator.apply_settings({"turnedOn": False, "velocity": 12000}) is True
    assert pk_instance.byte_writes == [(51, 40, 0)]
    assert actuator.get_settings()["turnedOn"] is False
    assert actuator.get_settings()["velocity"] == 12000


def test_feetech_actuator_calibrate_zero_writes_torque_128_and_resets_target(
    monkeypatch,
):
    ft_mod, act_mod = _load_modules()
    pk_instance = None

    def fake_sts(ph):
        nonlocal pk_instance
        pk_instance = FakeStsPacketHandler(ph)
        return pk_instance

    monkeypatch.setattr(ft_mod, "PortHandler", FakePortHandler)
    monkeypatch.setattr(ft_mod, "sts", fake_sts)
    ft_mod._port_cache.clear()

    actuator = act_mod.create_actuator("feetech_st_serial", 51, "WS-HEAD", "max")
    actuator._target_position = 1200
    actuator._settings["turnedOn"] = False
    assert actuator.calibrate_zero() is True
    assert pk_instance.byte_writes == [(51, 40, 128), (51, 40, 1)]
    assert pk_instance.writes == [(51, ft_mod.TICKS_AT_JOINT_ZERO, 700, 30)]
    assert actuator._target_position == 0
    assert actuator.get_settings()["turnedOn"] is True
    assert actuator.apply_settings({"calibrateZero": True}) is True
    assert pk_instance.byte_writes == [
        (51, 40, 128),
        (51, 40, 1),
        (51, 40, 128),
        (51, 40, 1),
    ]
    assert pk_instance.writes == [
        (51, ft_mod.TICKS_AT_JOINT_ZERO, 700, 30),
        (51, ft_mod.TICKS_AT_JOINT_ZERO, 700, 30),
    ]


def test_feetech_actuator_invert_preserves_sign_in_get_and_set(monkeypatch):
    ft_mod, act_mod = _load_modules()
    pk_instance = None

    def fake_sts(ph):
        nonlocal pk_instance
        pk_instance = FakeStsPacketHandler(ph)
        return pk_instance

    monkeypatch.setattr(ft_mod, "PortHandler", FakePortHandler)
    monkeypatch.setattr(ft_mod, "sts", fake_sts)
    ft_mod._port_cache.clear()

    inverted = act_mod.create_actuator("feetech_st_serial", 18, "WS-ARM", "min")
    normal = act_mod.create_actuator("feetech_st_serial", 11, "WS-ARM", "max")
    pk_instance = inverted._pk

    assert inverted.set_position(1000) is True
    assert normal.set_position(1000) is True
    assert inverted.get_position() == 1000
    assert normal.get_position() == 1000
    assert pk_instance.writes[0][1] != pk_instance.writes[1][1]


@pytest.mark.parametrize(
    ("upper_endpoint", "expected_tick_sign"),
    [
        ("max", 1),
        ("min", -1),
    ],
)
def test_feetech_drive_sign_matches_legacy_invert(
    monkeypatch, upper_endpoint, expected_tick_sign
):
    ft_mod, act_mod = _load_modules()
    from pib_motors.feetech_actuator import centideg_to_ticks

    def fake_sts(ph):
        return FakeStsPacketHandler(ph)

    monkeypatch.setattr(ft_mod, "PortHandler", FakePortHandler)
    monkeypatch.setattr(ft_mod, "sts", fake_sts)
    ft_mod._port_cache.clear()

    actuator = act_mod.create_actuator(
        "feetech_st_serial", 18, "WS-ARM", upper_endpoint
    )
    assert actuator.set_position(1000) is True
    expected_ticks = centideg_to_ticks(1000 * expected_tick_sign)
    assert actuator._pk.writes[-1][1] == expected_ticks


def test_feetech_actuator_custom_velocity_and_acceleration_in_write_pos_ex(
    monkeypatch,
):
    ft_mod, act_mod = _load_modules()
    pk_instance = None

    def fake_sts(ph):
        nonlocal pk_instance
        pk_instance = FakeStsPacketHandler(ph)
        return pk_instance

    monkeypatch.setattr(ft_mod, "PortHandler", FakePortHandler)
    monkeypatch.setattr(ft_mod, "sts", fake_sts)
    ft_mod._port_cache.clear()

    actuator = act_mod.create_actuator("feetech_st_serial", 18, "WS-ARM", "max")
    assert actuator.apply_settings({"velocity": 1200, "acceleration": 45}) is True
    assert actuator.set_position(0) is True
    assert pk_instance.writes[-1] == (18, ft_mod.TICKS_AT_JOINT_ZERO, 1200, 45)
