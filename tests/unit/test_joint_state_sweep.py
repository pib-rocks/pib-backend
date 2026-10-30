"""PR-2112: GroupSyncRead joint sweep and honest Feetech position semantics."""

from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from test_feetech_actuator import FakePortHandler, FakeStsPacketHandler, _load_modules

REPO_ROOT = Path(__file__).resolve().parents[2]
PIB_MOTORS_DIR = REPO_ROOT / "ros_packages" / "motors" / "pib_motors"
SWEEP_MODULE_PATH = PIB_MOTORS_DIR / "pib_motors" / "joint_state_sweep.py"

for path in (str(REPO_ROOT / "pib_api" / "client"), str(PIB_MOTORS_DIR)):
    if path not in sys.path:
        sys.path.insert(0, path)


def _load_sweep_module(ft_mod):
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "pib_motors.joint_state_sweep", SWEEP_MODULE_PATH
    )
    sweep_mod = importlib.util.module_from_spec(spec)
    sys.modules["pib_motors.joint_state_sweep"] = sweep_mod
    spec.loader.exec_module(sweep_mod)
    return sweep_mod


def test_get_position_reports_measured_not_stale_target(monkeypatch):
    """Load-bearing: stale commanded target must not appear as get_position()."""
    ft_mod, act_mod = _load_modules()

    def fake_sts(ph):
        return FakeStsPacketHandler(ph)

    monkeypatch.setattr(ft_mod, "PortHandler", FakePortHandler)
    monkeypatch.setattr(ft_mod, "sts", fake_sts)
    ft_mod._port_cache.clear()

    actuator = act_mod.create_actuator("feetech_st_serial", 51, "WS-HEAD", "max")
    assert actuator.set_position(5000) is True
    actuator._pk.pos_ticks_by_id[51] = ft_mod.TICKS_AT_JOINT_ZERO

    assert actuator.get_target_position() == 5000
    assert actuator.get_position() == 0


def test_sweep_returns_empty_when_no_feetech_actuators():
    sweep_mod = _load_sweep_module(_load_modules()[0])
    motor = SimpleNamespace(name="legacy", actuators=[])
    result = sweep_mod.sweep_feetech_joint_positions([motor])
    assert result.positions_centideg == {}
    assert result.ports_attempted == 0


def test_sweep_returns_empty_when_bus_disconnected(monkeypatch):
    ft_mod = _load_modules()[0]
    sweep_mod = _load_sweep_module(ft_mod)

    class OfflineActuator:
        kind = "feetech_st_serial"
        uid = "missing-adapter"

        def is_connected(self) -> bool:
            return False

    motor = SimpleNamespace(name="turn_head_motor", actuators=[OfflineActuator()])
    result = sweep_mod.sweep_feetech_joint_positions([motor])
    assert result.positions_centideg == {}
    assert result.ports_attempted == 0


def test_sweep_reads_every_servo_individually(monkeypatch):
    """Load-bearing: the .168 measurement - per-servo reads, distinct positions."""
    ft_mod, act_mod = _load_modules()
    sweep_mod = _load_sweep_module(ft_mod)

    def fake_sts(ph):
        pk = FakeStsPacketHandler(ph)
        pk.sts_tohost = lambda value, bit: int(value)
        pk.pos_ticks_by_id = {18: 2048, 11: 3072}
        return pk

    monkeypatch.setattr(ft_mod, "PortHandler", FakePortHandler)
    monkeypatch.setattr(ft_mod, "sts", fake_sts)
    ft_mod._port_cache.clear()

    elbow = act_mod.create_actuator("feetech_st_serial", 18, "WS-ARM", "max")
    thumb = act_mod.create_actuator("feetech_st_serial", 11, "WS-ARM", "max")
    motors = [
        SimpleNamespace(name="right_arm_motor_elbow", actuators=[elbow]),
        SimpleNamespace(name="right_hand_motor", actuators=[thumb]),
    ]

    result = sweep_mod.sweep_feetech_joint_positions(motors)

    assert set(result.positions_centideg) == {
        "right_arm_motor_elbow",
        "right_hand_motor",
    }
    assert (
        result.positions_centideg["right_arm_motor_elbow"]
        != result.positions_centideg["right_hand_motor"]
    )
    assert result.ports_succeeded == 1
    assert result.mode_by_port == {"WS-ARM": sweep_mod.MODE_SINGLE}
    assert result.port_durations_ms["WS-ARM"] >= 0.0


def test_failed_transactions_are_skipped_not_reported_as_zero(monkeypatch):
    """A servo answering with a bus error must not look like position 0."""
    ft_mod, act_mod = _load_modules()
    sweep_mod = _load_sweep_module(ft_mod)

    def fake_sts(ph):
        pk = FakeStsPacketHandler(ph)
        pk.sts_tohost = lambda value, bit: int(value)
        return pk

    monkeypatch.setattr(ft_mod, "PortHandler", FakePortHandler)
    monkeypatch.setattr(ft_mod, "sts", fake_sts)
    ft_mod._port_cache.clear()

    elbow = act_mod.create_actuator("feetech_st_serial", 18, "WS-ARM", "max")
    assert elbow.is_connected() is True
    elbow._pk.comm_result = 1
    motors = [SimpleNamespace(name="right_arm_motor_elbow", actuators=[elbow])]

    result = sweep_mod.sweep_feetech_joint_positions(motors)

    assert result.positions_centideg == {}
    assert result.ports_succeeded == 0


def test_sweep_probes_each_port_once_not_each_servo(monkeypatch):
    """Load-bearing: 26 probes cost 26 x 50 ms; one probe per port is enough."""
    ft_mod, act_mod = _load_modules()
    sweep_mod = _load_sweep_module(ft_mod)
    sweep_mod.forget_dead_ports()

    def fake_sts(ph):
        pk = FakeStsPacketHandler(ph)
        pk.sts_tohost = lambda value, bit: int(value)
        pk.pos_ticks_by_id = {16: 2048, 17: 2560, 18: 3072}
        return pk

    monkeypatch.setattr(ft_mod, "PortHandler", FakePortHandler)
    monkeypatch.setattr(ft_mod, "sts", fake_sts)
    ft_mod._port_cache.clear()

    motors = [
        SimpleNamespace(
            name=name,
            actuators=[act_mod.create_actuator("feetech_st_serial", pin, "WS", "max")],
        )
        for name, pin in (("j16", 16), ("j17", 17), ("j18", 18))
    ]
    probes = []
    for motor in motors:
        actuator = motor.actuators[0]
        original = actuator.is_connected

        def counting(original=original):
            probes.append(1)
            return original()

        actuator.is_connected = counting

    result = sweep_mod.sweep_feetech_joint_positions(motors)

    assert len(probes) == 1, f"eine Probe pro Port erwartet, waren {len(probes)}"
    assert result.ports_attempted == 1
    assert len(result.positions_centideg) == 3


def test_dead_port_is_not_probed_again_inside_the_ttl(monkeypatch):
    """A port that just failed must not cost its 50 ms timeout on every tick."""

    class OfflineActuator:
        kind = "feetech_st_serial"
        uid = "dead-adapter"

        def __init__(self):
            self.probes = 0

        def is_connected(self) -> bool:
            self.probes += 1
            return False

    sweep_mod = _load_sweep_module(_load_modules()[0])
    sweep_mod.forget_dead_ports()
    actuator = OfflineActuator()
    motors = [SimpleNamespace(name="turn_head_motor", actuators=[actuator])]

    first = sweep_mod.sweep_feetech_joint_positions(motors)
    second = sweep_mod.sweep_feetech_joint_positions(motors)

    assert actuator.probes == 1, f"nur eine Probe erwartet, waren {actuator.probes}"
    assert first.ports_unreachable == 1
    assert second.ports_unreachable == 1
    assert first.mode_by_port == {"dead-adapter": sweep_mod.MODE_UNREACHABLE}
    sweep_mod.forget_dead_ports()
