"""Pure OAK-D Lite IMU mapping, clock alignment and publication-rate decisions.

Everything numeric lives here and imports no ROS, so the host-side unit tests
exercise the same code the node runs.

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

from collections import deque
from dataclasses import dataclass
from datetime import timezone
import math

IMU_FRAME_ID = "oak_imu_frame"
IMU_PUBLISH_RATE_HZ = 100
IMU_PUBLISH_PERIOD_NS = 1_000_000_000 // IMU_PUBLISH_RATE_HZ
# One second of reports at the 200 Hz sensor rate. The offset estimate is a
# minimum over this window, which is a trade-off in both directions: a shorter
# window follows a single unusually fast report and is therefore noisy, a longer
# one keeps an offset the clocks have already drifted away from, because a
# minimum never rises again until the old sample leaves the window. One second
# keeps the estimate insensitive to a single outlier while bounding its error to
# the relative drift of two crystal clocks over one second, which is orders of
# magnitude below the report interval.
IMU_CLOCK_OFFSET_WINDOW = 200
# How far a stamp built from the device clock may sit from the host receipt time
# before it is discarded as implausible. The real distance is the transport
# latency of the report, at most the device queue depth in time (100 ms), plus
# the clock drift accumulated over the offset window; one second is generous for
# both and still rejects every stamp that is not host time at all.
IMU_STAMP_TOLERANCE_NS = 1_000_000_000
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


class PublishRateThrottle:
    """Accept samples one publication period apart and drop the rest."""

    def __init__(self):
        self._next_stamp_ns = None

    def accept(self, stamp_ns):
        accepted, self._next_stamp_ns = throttle_decision(stamp_ns, self._next_stamp_ns)
        return accepted


def throttle_decision(stamp_ns, next_stamp_ns):
    """Return an immutable accept/drop decision and the next state.

    One sample per ``IMU_PUBLISH_PERIOD_NS`` is accepted, so the 200 Hz sensor
    stream becomes the 100 Hz publication stream.

    Callers pass the device monotonic timestamp, which the sensor spaces
    uniformly: throttling on host receipt stamps instead lets the drain
    quantisation push samples just under the deadline, which was measured at
    8.1 Hz instead of the intended 10 Hz before this throttle existed.
    """

    stamp_ns = int(stamp_ns)
    if next_stamp_ns is None or stamp_ns >= next_stamp_ns:
        return True, stamp_ns + IMU_PUBLISH_PERIOD_NS
    return False, next_stamp_ns


def clock_offset_sample_ns(host_receipt_ns, device_stamp_ns):
    """Return one observation of host clock minus device clock, in nanoseconds."""

    return int(host_receipt_ns) - int(device_stamp_ns)


def estimated_clock_offset_ns(samples, window=IMU_CLOCK_OFFSET_WINDOW):
    """Estimate the device-to-host clock offset as the window MINIMUM.

    Each sample is ``host receipt - device timestamp``, so it carries the real
    offset plus the transport and drain latency of that one report. That latency
    is non-negative and unbounded above - batching, USB scheduling and the poll
    timer can only make a report arrive later, never earlier - so the smallest
    observation in the window is the one with the least added latency and is the
    best available estimate. A mean would instead track the average queueing
    delay and shift every stamp into the future by it.

    Returns ``None`` while fewer than two usable samples are available: with a
    single observation there is nothing to take a minimum over, and the caller
    must then fall back to the host receipt time. Values that are not finite
    numbers are ignored; negative offsets are kept, because the device clock
    counts from device boot and may well be ahead of the host epoch.
    """

    window = int(window)
    if window < 2:
        raise ValueError("clock offset window must hold at least two samples")
    usable = []
    # Kept as integers: nanosecond offsets against the host epoch are around
    # 1.7e18, where float64 already quantises in steps of hundreds of
    # nanoseconds.
    for value in list(samples)[-window:]:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            continue
        if isinstance(value, float) and not math.isfinite(value):
            continue
        usable.append(int(value))
    if len(usable) < 2:
        return None
    return min(usable)


def published_stamp_ns(
    device_stamp_ns,
    host_receipt_ns,
    offset_ns,
    tolerance_ns=IMU_STAMP_TOLERANCE_NS,
):
    """Return the header stamp: the measurement instant on the host timeline.

    With an established offset the stamp is ``device timestamp + offset``, which
    inherits the uniform spacing of the sensor's own sampling instead of the
    jitter of the host drain. Because the offset is the window minimum, the
    result is never later than the receipt time of the report that produced that
    minimum, so this cannot stamp a measurement into the future.

    Without an offset - and whenever the shifted stamp lands further than
    ``tolerance_ns`` from the receipt time - the host receipt time is published:
    late by the transport latency, but real host time. That tolerance is what
    makes "a device duration is never published as wall time" structural rather
    than a promise: a raw device timestamp is a duration since device boot and
    differs from host epoch nanoseconds by decades, so no offset that fails to
    map it into host time can survive the comparison. The same check catches a
    device clock that restarted while an offset from the previous session was
    still in the window.
    """

    host_receipt_ns = int(host_receipt_ns)
    if offset_ns is None:
        return host_receipt_ns
    stamp_ns = int(device_stamp_ns) + int(offset_ns)
    if stamp_ns <= 0 or abs(stamp_ns - host_receipt_ns) > int(tolerance_ns):
        return host_receipt_ns
    return stamp_ns


class ClockOffsetEstimator:
    """Sliding window of offset observations behind ``estimated_clock_offset_ns``."""

    def __init__(self, window=IMU_CLOCK_OFFSET_WINDOW):
        self._window = int(window)
        self._samples = deque(maxlen=self._window)

    def observe(self, host_receipt_ns, device_stamp_ns):
        """Record one report and return the current offset estimate, or None."""

        self._samples.append(clock_offset_sample_ns(host_receipt_ns, device_stamp_ns))
        return self.offset_ns()

    def offset_ns(self):
        return estimated_clock_offset_ns(self._samples, self._window)


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
    """Return the host-time instant at which this report became available.

    Measured on depthai 3.6.1 (the build shipped in the camera container): the
    IMU reports expose ``getTimestamp()`` (a ``timedelta`` "related to
    dai::Clock::now()", so not an epoch stamp), ``getTimestampDevice()`` (device
    monotonic clock, explicitly not synchronized to host time) and
    ``getSequenceNum()``. ``getTimestampSystem()``, which newer documentation
    shows, does not exist here. An epoch-valued host timestamp is therefore used
    when the report offers one, and otherwise the host time at which the node
    received the report - never a device duration reinterpreted as wall time.

    This value is the receipt side of the offset estimate, and the published
    stamp whenever no offset is established yet; ``published_stamp_ns`` decides
    between the two.
    """

    if hasattr(report_timestamp, "timestamp"):
        return duration_to_nanoseconds(report_timestamp)
    return int(round(float(host_now_seconds) * 1_000_000_000))


def measured_rate_hz(timestamps):
    """Return the publication rate observed from monotonic timestamps.

    The status channel reports this instead of the configured rate: the live rig
    measured 8.1 Hz while the configuration said 10 Hz, so a configured constant
    would have been a value nobody measured, and the same applies to the 100 Hz
    configured now. Fewer than two samples or a non-positive span yields 0.0
    rather than a guess.
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
