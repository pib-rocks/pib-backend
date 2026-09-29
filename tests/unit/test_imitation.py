"""Device-free tests for hand mapping, IK, and trajectory safety gates."""

from types import SimpleNamespace

import pytest

from ros_packages.imitation.imitation.landmark_mapping import (
    HAND_LANDMARK_NAMES,
    map_to_robot_target,
    palm_pose_from_landmarks,
)
from ros_packages.imitation.imitation.trajectory import (
    TeleopController,
    build_timed_trajectory,
)
from ros_packages.motors.pib_motors.pib_motors.trajectory_executor import (
    is_software_trajectory,
    parse_joint_trajectory,
)


def synthetic_landmarks(depth=600.0):
    coordinates = {name: (100.0, 100.0) for name in HAND_LANDMARK_NAMES}
    coordinates.update(
        {
            "wrist": (100.0, 140.0),
            "index_finger_mcp": (140.0, 100.0),
            "middle_finger_mcp": (100.0, 80.0),
            "pinky_mcp": (60.0, 100.0),
        }
    )
    names = list(reversed(HAND_LANDMARK_NAMES))
    return (
        names,
        [coordinates[name][0] for name in names],
        [coordinates[name][1] for name in names],
        [depth for _ in names],
    )


def test_named_landmarks_are_reordered_and_produce_expected_palm_geometry():
    pose = palm_pose_from_landmarks(
        *synthetic_landmarks(), frame_width=200, frame_height=200
    )

    assert pose.position.x == pytest.approx(0.0)
    assert pose.position.y == pytest.approx(5.0)
    assert pose.position.z == pytest.approx(600.0)
    assert pose.scale_mm_per_pixel == pytest.approx(1.0)
    assert pose.quaternion_xyzw == pytest.approx((1.0, 0.0, 0.0, 0.0))
    assert pose.used_depth_fallback is False


@pytest.mark.parametrize("invalid_depth", [0.0, float("nan"), -1.0])
def test_zero_or_invalid_depth_uses_documented_fallback(invalid_depth):
    pose = palm_pose_from_landmarks(
        *synthetic_landmarks(invalid_depth),
        frame_width=200,
        frame_height=200,
        fallback_depth_mm=475.0,
    )

    assert pose.position.z == pytest.approx(475.0)
    assert pose.used_depth_fallback is True


def test_valid_depth_is_used_for_zero_entries_before_global_fallback():
    names, xs, ys, zs = synthetic_landmarks(0.0)
    zs[names.index("middle_finger_mcp")] = 640.0
    pose = palm_pose_from_landmarks(
        names, xs, ys, zs, frame_width=200, frame_height=200
    )

    assert pose.position.z == pytest.approx(640.0)
    assert pose.used_depth_fallback is True


def test_parallel_array_and_name_validation():
    names, xs, ys, zs = synthetic_landmarks()
    with pytest.raises(ValueError, match="parallel 21"):
        palm_pose_from_landmarks(
            names, xs[:-1], ys, zs, frame_width=200, frame_height=200
        )
    names[0] = names[1]
    with pytest.raises(ValueError, match="each expected"):
        palm_pose_from_landmarks(names, xs, ys, zs, frame_width=200, frame_height=200)


def test_camera_pose_maps_to_robot_axes_deterministically():
    hand = palm_pose_from_landmarks(
        *synthetic_landmarks(), frame_width=200, frame_height=200
    )
    target = map_to_robot_target(
        hand, origin_mm=(180.0, -180.0, 220.0), reference_depth_mm=500.0
    )

    assert target.position.x == pytest.approx(80.0)
    assert target.position.y == pytest.approx(-180.0)
    assert target.position.z == pytest.approx(215.0)


class FakeKinematics:
    motor_names = ("joint_a", "joint_b")
    joint_limits_deg = ([-90.0, -45.0], [90.0, 45.0])

    def __init__(self):
        self.solution = (10.125, -20.125)
        self.calls = []

    def inverse(self, **kwargs):
        self.calls.append(kwargs)
        return self.solution


class FakeFuture:
    def __init__(self):
        self.callbacks = []

    def add_done_callback(self, callback):
        self.callbacks.append(callback)

    def finish(self):
        for callback in self.callbacks:
            callback(self)


class FakeClient:
    def __init__(self):
        self.requests = []

    def call_async(self, request):
        self.requests.append(request)
        return FakeFuture()


def test_motion_disabled_never_calls_trajectory_client():
    client = FakeClient()
    controller = TeleopController(FakeKinematics(), client, enable_motion=False)

    assert controller.prepare((1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0)) is None
    assert controller.submit(object()) is None
    assert client.requests == []


def test_ik_limits_timed_conversion_and_non_overlapping_requests():
    kinematics = FakeKinematics()
    client = FakeClient()
    controller = TeleopController(
        kinematics, client, enable_motion=True, duration_sec=0.2
    )

    # First solution only establishes a baseline; it cannot invent current state.
    assert controller.prepare((1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0)) is None
    kinematics.solution = (11.0, -19.0)
    trajectory = controller.prepare((2.0, 3.0, 4.0), (0.0, 0.0, 0.0, 1.0))
    assert trajectory.joint_names == kinematics.motor_names
    assert trajectory.waypoints[0].positions == (1012.0, -2012.0)
    assert trajectory.waypoints[1].positions == (1100.0, -1900.0)
    assert [point.time_from_start for point in trajectory.waypoints] == [0.0, 0.2]

    future = controller.submit(trajectory)
    assert controller.request_in_flight is True
    assert controller.submit(trajectory) is None
    assert len(client.requests) == 1
    future.finish()
    assert controller.request_in_flight is False


def test_unreachable_or_nonfinite_ik_targets_are_rejected():
    kinematics = FakeKinematics()
    controller = TeleopController(kinematics, FakeClient(), enable_motion=True)
    with pytest.raises(ValueError, match="finite coordinates"):
        controller.prepare((float("nan"), 2.0, 3.0), (0.0, 0.0, 0.0, 1.0))
    kinematics.solution = (100.0, 0.0)
    with pytest.raises(ValueError, match="mechanical joint limits"):
        controller.prepare((1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0))


def test_timed_layout_is_compatible_with_trajectory_executor():
    trajectory = build_timed_trajectory(
        ("joint_a", "joint_b"), (1.0, 2.0), (3.0, 4.0), duration_sec=0.25
    )
    message = SimpleNamespace(
        joint_names=list(trajectory.joint_names),
        points=[
            SimpleNamespace(
                positions=list(point.positions),
                time_from_start=point.time_from_start,
            )
            for point in trajectory.waypoints
        ],
    )

    assert is_software_trajectory(message) is True
    names, times, series = parse_joint_trajectory(message)
    assert names == ["joint_a", "joint_b"]
    assert times == [0.0, 0.25]
    assert series == {"joint_a": [100.0, 300.0], "joint_b": [200.0, 400.0]}
