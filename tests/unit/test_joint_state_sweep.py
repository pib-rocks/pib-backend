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
