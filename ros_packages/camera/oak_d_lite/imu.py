"""Pure OAK-D Lite IMU mapping and publication-rate decisions.

DepthAI reports the IMU in the camera optical basis (right, down, forward).
ROS REP-103 body axes are forward, left, up, so vectors are mapped as
``(z, -x, -y)``.

Orientation is deliberately not produced here. The BMI270 on the OAK-D Lite
refuses every fused output - the device answers a ROTATION_VECTOR request with
``IMU invalid settings!: ROTATION_VECTOR output is unsupported. BMI270 supports
only ACCELEROMETER_RAW and/or GYROSCOPE_RAW outputs.`` - so a quaternion would
have to be fused on the host, which is a separate decision. The published
message therefore marks orientation as unavailable instead of inventing one.
"""

from dataclasses import dataclass
from datetime import timezone
import math

IMU_FRAME_ID = "oak_imu_frame"
IMU_PUBLISH_RATE_HZ = 10
IMU_PUBLISH_PERIOD_NS = 1_000_000_000 // IMU_PUBLISH_RATE_HZ
UNKNOWN_COVARIANCE = (-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


@dataclass(frozen=True)
class Vector3:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class ImuSample:
    """ROS-ready values without depending on ROS message classes."""

    frame_id: str
    sequence: int
    stamp_ns: int
    linear_acceleration: Vector3
    angular_velocity: Vector3
    orientation_covariance: tuple
    angular_velocity_covariance: tuple
    linear_acceleration_covariance: tuple


class TenHertzThrottle:
    """Accept samples at least 100 ms apart and deterministically drop the rest."""

    def __init__(self):
        self._next_stamp_ns = None

    def accept(self, stamp_ns):
        accepted, self._next_stamp_ns = throttle_decision(stamp_ns, self._next_stamp_ns)
        return accepted


def throttle_decision(stamp_ns, next_stamp_ns):
    """Return an immutable 10 Hz accept/drop decision and the next state.

    Callers pass the device monotonic timestamp, which the sensor spaces
    uniformly: throttling on host receipt stamps instead lets the drain
    quantisation push samples just under the deadline, which was measured at
    8.1 Hz instead of the intended 10 Hz.
    """

    stamp_ns = int(stamp_ns)
    if next_stamp_ns is None or stamp_ns >= next_stamp_ns:
        return True, stamp_ns + IMU_PUBLISH_PERIOD_NS
    return False, next_stamp_ns


def optical_to_rep103(x, y, z):
    """Map right/down/forward camera coordinates to forward/left/up."""

    return Vector3(float(z), -float(x), -float(y))


def duration_to_nanoseconds(timestamp):
    """Convert a DepthAI host timestamp (datetime/timedelta-like) to nanoseconds."""

    if hasattr(timestamp, "timestamp"):
        if getattr(timestamp, "tzinfo", None) is None:
            timestamp = timestamp.replace(tzinfo=timezone.utc)
        seconds = timestamp.timestamp()
    elif hasattr(timestamp, "total_seconds"):
        seconds = timestamp.total_seconds()
    else:
        seconds = float(timestamp)
    if not math.isfinite(seconds) or seconds < 0:
        raise ValueError("IMU timestamp must be finite and non-negative")
    return int(round(seconds * 1_000_000_000))


def host_stamp_nanoseconds(report_timestamp, host_now_seconds):
    """Return the ROS header stamp in host time.

    Measured on depthai 3.6.1 (the build shipped in the camera container): the
    IMU reports expose ``getTimestamp()`` (a ``timedelta`` "related to
    dai::Clock::now()", so not an epoch stamp), ``getTimestampDevice()`` (device
    monotonic clock, explicitly not synchronized to host time) and
    ``getSequenceNum()``. ``getTimestampSystem()``, which newer documentation
    shows, does not exist here. An epoch-valued host timestamp is therefore used
    when the report offers one, and otherwise the host time at which the node
    received the report - never a device duration reinterpreted as wall time.
    """

    if hasattr(report_timestamp, "timestamp"):
        return duration_to_nanoseconds(report_timestamp)
    return int(round(float(host_now_seconds) * 1_000_000_000))


def measured_rate_hz(timestamps):
    """Return the publication rate observed from monotonic timestamps.

    The status channel reports this instead of the configured rate: the live rig
    measured 8.1 Hz while the configuration said 10 Hz, so a configured constant
    would have been a value nobody measured. Fewer than two samples or a
    non-positive span yields 0.0 rather than a guess.
    """

    if len(timestamps) < 2:
        return 0.0
    span = float(timestamps[-1]) - float(timestamps[0])
    if not math.isfinite(span) or span <= 0.0:
        return 0.0
    return (len(timestamps) - 1) / span


def assemble_imu_sample(
    sequence,
    stamp_ns,
    acceleration_xyz,
    angular_velocity_xyz,
):
    """Build one REP-103 sample from DepthAI's SI-valued sensor reports."""

    return ImuSample(
        frame_id=IMU_FRAME_ID,
        sequence=int(sequence),
        stamp_ns=int(stamp_ns),
        # DepthAI supplies accelerometer values in m/s² and gyro values in rad/s.
        linear_acceleration=optical_to_rep103(*acceleration_xyz),
        angular_velocity=optical_to_rep103(*angular_velocity_xyz),
        # The BMI270 reports no covariance and no fused orientation. -1 in
        # element zero is the sensor_msgs/Imu convention for "unavailable", so
        # the orientation field is explicitly marked unknown rather than filled
        # with a value the device never measured.
        orientation_covariance=UNKNOWN_COVARIANCE,
        angular_velocity_covariance=UNKNOWN_COVARIANCE,
        linear_acceleration_covariance=UNKNOWN_COVARIANCE,
    )
