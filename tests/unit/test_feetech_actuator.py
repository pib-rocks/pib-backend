"""PC-125: unit tests for FeetechStSerialActuator and serial adapter resolution."""

from __future__ import annotations

import importlib.util
import sys
import time
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


@pytest.fixture(autouse=True)
def _reset_bus_writer_registry():
    from pib_motors.bus_writer import reset_bus_writer_registry_for_tests

    reset_bus_writer_registry_for_tests()
    yield
    reset_bus_writer_registry_for_tests()


class FakeStsPacketHandler:
    """Fakes pib_motors.stservo_sdk.sts.sts without opening a physical serial port."""

    def __init__(self, ph) -> None:
        self.ph = ph
        self.pos_ticks: int = 2048
        self.pos_ticks_by_id: dict[int, int] = {}
        self.speed: int = 0
        self.current: int = 15
        self.temperature: int = 34
        self.voltage_raw: int = 118
        self.comm_result: int = 0  # COMM_SUCCESS
        self.writes: list[tuple[int, int, int, int]] = []
        self.byte_writes: list[tuple[int, int, int]] = []

    def ReadPosSpeed(self, servo_id: int):
        ticks = self.pos_ticks_by_id.get(servo_id, self.pos_ticks)
        return ticks, self.speed, self.comm_result, 0

    def ReadCurrent(self, servo_id: int):
        return self.current, self.comm_result, 0

    def WritePosEx(self, servo_id: int, position: int, speed: int, acc: int):
        self.writes.append((servo_id, position, speed, acc))
        if self.comm_result == 0:
            self.pos_ticks = position
            self.pos_ticks_by_id[servo_id] = position
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
    assert inverted.get_target_position() == 1000
    assert normal.get_target_position() == 1000
    assert inverted.get_position() == inverted.get_current_position()
    assert normal.get_position() == normal.get_current_position()
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


def test_holding_the_port_too_long_is_logged_with_its_caller(
    monkeypatch, caplog
) -> None:
    """The holder of the port is invisible otherwise: every waiter only sees the wait.

    With the threshold lowered to 1 ms, a short artificial hold must produce a
    BUS_ACCESS_HELD line naming the function that held it.
    """
    import threading

    from pib_motors import feetech_actuator

    class _FakePort:
        port_name = "/dev/fake-motor0"
        ser = None

    monkeypatch.setattr(feetech_actuator, "HELD_WARN_MS", 1.0)
    with caplog.at_level("WARNING"):
        with feetech_actuator._BusAccess(_FakePort(), threading.Lock()):
            time.sleep(0.01)
    assert "BUS_ACCESS_HELD" in caplog.text
    assert "test_holding_the_port_too_long_is_logged_with_its_caller" in caplog.text


def test_short_port_hold_is_not_logged(monkeypatch, caplog) -> None:
    """Counter-check: without this the warning above could fire on every transaction."""
    import threading

    from pib_motors import feetech_actuator

    class _FakePort:
        port_name = "/dev/fake-motor0"
        ser = None

    monkeypatch.setattr(feetech_actuator, "HELD_WARN_MS", 5_000.0)
    with caplog.at_level("WARNING"):
        with feetech_actuator._BusAccess(_FakePort(), threading.Lock()):
            time.sleep(0.005)
    assert "BUS_ACCESS_HELD" not in caplog.text


def test_tty_flags_keep_dtr_and_drop_hardware_flow_control() -> None:
    """Three processes share each adapter; the last close must not drop DTR.

    Measured on the pib5advanced: when an adapter stops answering there is no USB disconnect in
    dmesg, so the link is fine and only the line state changes. HUPCL - the default - drops DTR
    on the last close, which a CDC-ACM bridge firmware reads as "host gone".
    """
    import termios

    from pib_motors import feetech_actuator

    # no CLOCAL in the input: otherwise setting it could not be observed
    cflag = termios.HUPCL | termios.CRTSCTS | termios.CREAD
    result = feetech_actuator.desired_cflag(cflag)

    assert result & termios.HUPCL == 0, "HUPCL must be cleared"
    assert result & termios.CRTSCTS == 0, "hardware flow control must be off"
    assert result & termios.CLOCAL, "CLOCAL must be set"
    assert result & termios.CREAD, "unrelated flags must survive"


def test_tty_flags_are_idempotent() -> None:
    """Applying the flags twice must not accumulate anything."""
    import termios

    from pib_motors import feetech_actuator

    once = feetech_actuator.desired_cflag(termios.CLOCAL)
    assert feetech_actuator.desired_cflag(once) == once


def test_reopen_port_keeps_existing_actuator_and_bus_writer_handles_alive(
    monkeypatch,
) -> None:
    """PR-2159: reopen_port() must reopen the cached PortHandler in place instead of
    popping it and leaving BusWriter._ph / actuator._ph closed forever.
    """
    ft_mod, act_mod = _load_modules()

    class StrictPortHandler(FakePortHandler):
        def __init__(self, port_name: str, can_open: bool = True) -> None:
            super().__init__(port_name, can_open=can_open)
            self.open_count = 0
            self.close_count = 0

        def openPort(self) -> bool:
            self.open_count += 1
            return super().openPort()

        def closePort(self) -> None:
            self.close_count += 1
            super().closePort()

    class StrictPacketHandler(FakeStsPacketHandler):
        def ReadPosSpeed(self, servo_id: int):
            if not self.ph.is_open:
                raise RuntimeError("serial port is closed")
            return super().ReadPosSpeed(servo_id)

        def WritePosEx(self, servo_id: int, position: int, speed: int, acc: int):
            if not self.ph.is_open:
                raise RuntimeError("serial port is closed")
            return super().WritePosEx(servo_id, position, speed, acc)

    monkeypatch.setattr(ft_mod, "PortHandler", StrictPortHandler)
    monkeypatch.setattr(ft_mod, "sts", lambda ph: StrictPacketHandler(ph))
    ft_mod._port_cache.clear()

    actuator = act_mod.create_actuator("feetech_st_serial", 51, "5B91030562", "max")
    ph_before = actuator._ph
    assert ph_before.is_open is True
    assert actuator.set_position(1000) is True

    # Trigger reopen_port: the SAME PortHandler instance must be closed and reopened in place
    assert ft_mod.reopen_port("5B91030562") is True
    assert actuator._ph is ph_before
    assert ph_before.close_count == 1
    assert ph_before.open_count == 2
    assert ph_before.is_open is True

    # Existing actuator and BusWriter continue working without recreating anything
    assert actuator.set_position(4500) is True
    assert actuator.get_position() == 4500


def test_get_temperature_and_get_voltage_route_through_bus_writer(
    monkeypatch,
) -> None:
    """PR-2159: get_temperature() and get_voltage() must serialize through BusWriter.run_service."""
    ft_mod, act_mod = _load_modules()

    def fake_sts(ph):
        pk = FakeStsPacketHandler(ph)
        pk.temperature = 41
        pk.voltage_raw = 123
        return pk

    monkeypatch.setattr(ft_mod, "PortHandler", FakePortHandler)
    monkeypatch.setattr(ft_mod, "sts", fake_sts)
    ft_mod._port_cache.clear()

    actuator = act_mod.create_actuator("feetech_st_serial", 51, "WS-HEAD", "max")
    writer = actuator._writer()
    labels: list[str] = []
    orig_run_service = writer.run_service

    def spy_run_service(execute, *, deadline=None, wait_timeout=None, label=""):
        labels.append(label)
        return orig_run_service(
            execute, deadline=deadline, wait_timeout=wait_timeout, label=label
        )

    monkeypatch.setattr(writer, "run_service", spy_run_service)

    assert actuator.get_temperature() == 41
    assert actuator.get_voltage() == 12.3
    assert "pin51:get_temperature" in labels
    assert "pin51:get_voltage" in labels


def test_bus_access_clears_wedged_is_using_flag_on_exit() -> None:
    """PR-2159: if stservo_sdk leaves PortHandler.is_using=True after an interrupted
    transaction, _BusAccess.__exit__ must clear it so subsequent transactions do not fail
    immediately with COMM_PORT_BUSY (-1).
    """
    import threading
    from pib_motors import feetech_actuator

    class _FakePort:
        port_name = "/dev/fake-motor0"
        ser = None
        is_using = True

    ph = _FakePort()
    with feetech_actuator._BusAccess(ph, threading.Lock()):
        assert ph.is_using is True
    assert ph.is_using is False
