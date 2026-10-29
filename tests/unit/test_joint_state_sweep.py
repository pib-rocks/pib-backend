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


def test_group_sync_read_populates_multiple_joints_on_one_port(monkeypatch):
    ft_mod, act_mod = _load_modules()
    sweep_mod = _load_sweep_module(ft_mod)

    class RecordingGroupSyncRead:
        instances: list["RecordingGroupSyncRead"] = []

        def __init__(self, ph, start_address, data_length):
            self.ph = ph
            self.data_dict = {}
            self.last_result = True
            RecordingGroupSyncRead.instances.append(self)

        def addParam(self, sts_id):
            self.data_dict[sts_id] = []
            return True

        def txRxPacket(self):
            return 0

        def isAvailable(self, sts_id, address, data_length):
            return True, 0

        def getData(self, sts_id, address, data_length):
            return {18: 2560, 11: 2560}[sts_id]

    monkeypatch.setattr(sweep_mod, "GroupSyncRead", RecordingGroupSyncRead)

    def fake_sts(ph):
        pk = FakeStsPacketHandler(ph)
        pk.sts_tohost = lambda value, bit: int(value)
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
    assert len(RecordingGroupSyncRead.instances) == 1
    assert set(result.positions_centideg) == {
        "right_arm_motor_elbow",
        "right_hand_motor",
    }


def _fallback_fixture(monkeypatch, *, group_answers: bool, single_comm: int = 0):
    """Build a two-servo port whose group read may or may not answer."""
    ft_mod, act_mod = _load_modules()
    sweep_mod = _load_sweep_module(ft_mod)
    sweep_mod._SINGLE_READ_PORTS.clear()

    class RecordingGroupSyncRead:
        instances: list["RecordingGroupSyncRead"] = []

        def __init__(self, ph, start_address, data_length):
            self.data_dict = {}
            self.last_result = group_answers
            RecordingGroupSyncRead.instances.append(self)

        def addParam(self, sts_id):
            self.data_dict[sts_id] = []
            return True

        def txRxPacket(self):
            return 0 if group_answers else 1

        def isAvailable(self, sts_id, address, data_length):
            return group_answers, 0

        def getData(self, sts_id, address, data_length):
            return 2048

    monkeypatch.setattr(sweep_mod, "GroupSyncRead", RecordingGroupSyncRead)

    handler = {}

    def fake_sts(ph):
        pk = FakeStsPacketHandler(ph)
        pk.sts_tohost = lambda value, bit: int(value)
        pk.comm_result = single_comm
        # Distinct per-servo ticks prove the fallback reads each ID separately.
        pk.pos_ticks_by_id = {18: 2048, 11: 3072}
        handler["pk"] = pk
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
    return sweep_mod, RecordingGroupSyncRead, motors


def test_port_falls_back_to_single_reads_when_the_group_read_is_silent(monkeypatch):
    """Load-bearing: the .168 case - broadcast unanswered, servos reachable one by one."""
    sweep_mod, _, motors = _fallback_fixture(monkeypatch, group_answers=False)

    result = sweep_mod.sweep_feetech_joint_positions(motors)

    assert set(result.positions_centideg) == {
        "right_arm_motor_elbow",
        "right_hand_motor",
    }
    # Different servos, different measured positions - proof of separate reads.
    assert (
        result.positions_centideg["right_arm_motor_elbow"]
        != result.positions_centideg["right_hand_motor"]
    )
    assert result.ports_succeeded == 1
    assert set(result.mode_by_port.values()) == {sweep_mod.MODE_SINGLE}
    assert sweep_mod.single_read_ports() == {"WS-ARM"}
    assert result.port_durations_ms["WS-ARM"] >= 0.0


def test_remembered_port_skips_the_group_read_on_the_next_sweep(monkeypatch):
    """Load-bearing: the slow port must not pay a failed broadcast every tick."""
    sweep_mod, group_reads, motors = _fallback_fixture(monkeypatch, group_answers=False)

    sweep_mod.sweep_feetech_joint_positions(motors)
    attempts_after_first = len(group_reads.instances)
    second = sweep_mod.sweep_feetech_joint_positions(motors)

    assert attempts_after_first == 1
    assert len(group_reads.instances) == attempts_after_first
    assert set(second.positions_centideg) == {
        "right_arm_motor_elbow",
        "right_hand_motor",
    }
    assert second.mode_by_port == {"WS-ARM": sweep_mod.MODE_SINGLE}


def test_silent_single_reads_are_not_remembered_as_working(monkeypatch):
    """A port nobody answers stays unmeasured and keeps trying the fast path."""
    sweep_mod, group_reads, motors = _fallback_fixture(monkeypatch, group_answers=False)
    # Reachable port whose transactions then fail: the servos stop answering
    # after the connection probe, which is the interesting case for the memory.
    for motor in motors:
        for actuator in motor.actuators:
            assert actuator.is_connected() is True
            actuator._pk.comm_result = 1

    result = sweep_mod.sweep_feetech_joint_positions(motors)

    assert result.positions_centideg == {}
    assert result.ports_succeeded == 0
    assert sweep_mod.single_read_ports() == set()
    sweep_mod.sweep_feetech_joint_positions(motors)
    assert len(group_reads.instances) == 2


def test_fallback_still_runs_when_the_group_read_throws(monkeypatch):
    """Load-bearing: .168 case - the broadcast raised, so the old code returned empty."""
    ft_mod, act_mod = _load_modules()
    sweep_mod = _load_sweep_module(ft_mod)
    sweep_mod._SINGLE_READ_PORTS.clear()

    class ThrowingGroupSyncRead:
        def __init__(self, ph, start_address, data_length):
            raise RuntimeError("broadcast not supported by this adapter")

    monkeypatch.setattr(sweep_mod, "GroupSyncRead", ThrowingGroupSyncRead)

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
    assert result.mode_by_port == {"WS-ARM": sweep_mod.MODE_SINGLE}
    assert sweep_mod.single_read_ports() == {"WS-ARM"}
