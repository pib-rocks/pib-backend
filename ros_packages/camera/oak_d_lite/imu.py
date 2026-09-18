"""Pure OAK-D Lite IMU mapping and publication-rate decisions.

DepthAI reports the IMU in the camera optical basis (right, down, forward).
ROS REP-103 body axes are forward, left, up, so vectors and the vector part of
the device-fused quaternion are mapped as ``(z, -x, -y)``.
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
class Quaternion:
    x: float
    y: float
    z: float
    w: float


@dataclass(frozen=True)
class ImuSample:
    """ROS-ready values without depending on ROS message classes."""

    frame_id: str
    sequence: int
    stamp_ns: int
    linear_acceleration: Vector3
    angular_velocity: Vector3
    orientation: Quaternion
    orientation_is_device_fused: bool
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
    """Return an immutable 10 Hz accept/drop decision and the next state."""

    stamp_ns = int(stamp_ns)
    if next_stamp_ns is None or stamp_ns >= next_stamp_ns:
        return True, stamp_ns + IMU_PUBLISH_PERIOD_NS
    return False, next_stamp_ns


def optical_to_rep103(x, y, z):
    """Map right/down/forward camera coordinates to forward/left/up."""

    return Vector3(float(z), -float(x), -float(y))


def normalized_rep103_quaternion(x, y, z, w):
    """Map and normalize the device-fused rotation-vector quaternion."""

    mapped = (float(z), -float(x), -float(y), float(w))
    norm = math.sqrt(sum(component * component for component in mapped))
    if not math.isfinite(norm) or norm <= 0.0:
        raise ValueError("rotation-vector quaternion must have a finite non-zero norm")
    return Quaternion(*(component / norm for component in mapped))


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


def assemble_imu_sample(
    sequence,
    stamp_ns,
    acceleration_xyz,
    angular_velocity_xyz,
    rotation_xyzw,
):
    """Build one REP-103 sample from DepthAI's SI-valued sensor reports."""

    return ImuSample(
        frame_id=IMU_FRAME_ID,
        sequence=int(sequence),
        stamp_ns=int(stamp_ns),
        # DepthAI supplies accelerometer values in m/s² and gyro values in rad/s.
        linear_acceleration=optical_to_rep103(*acceleration_xyz),
        angular_velocity=optical_to_rep103(*angular_velocity_xyz),
        orientation=normalized_rep103_quaternion(*rotation_xyzw),
        orientation_is_device_fused=True,
        # The BMI270 reports no covariance. -1 in element zero is the
        # sensor_msgs/Imu convention for an unavailable covariance estimate.
        orientation_covariance=UNKNOWN_COVARIANCE,
        angular_velocity_covariance=UNKNOWN_COVARIANCE,
        linear_acceleration_covariance=UNKNOWN_COVARIANCE,
    )
