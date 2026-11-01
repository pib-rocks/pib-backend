"""Measured speed characteristic of the STS servos, and the inverse for path coupling.

The STS ``WritePosEx`` speed setting is NOT a linear rate. Measured on the pib5advanced at
192.168.178.88 by commanding a single-point move at a fixed speed setting and taking the cruise
slope of the position series (``/joint_states``, about 10 Hz sampling, so the slope is an average
over roughly 200 ms and slightly smoothed by the servo ramp):

===================================  ==================  ===========================
servo family (pins)                  speed setting        measured cruise
===================================  ==================  ===========================
STS3095 (18-21, 38-41, 50, 51)       200                  2091.8 centideg/s
                                     700                  4729.3 centideg/s
                                     2000                 4948.8 centideg/s
fingers (everything else)            200                  1740.5 centideg/s
                                     700                  3483.3 centideg/s
                                     2000                 3834.6 centideg/s
===================================  ==================  ===========================

Two things follow from that table and both matter for the trajectory code:

* the relation is strongly sub-linear and then saturates - between 700 and 2000 the finger
  servo gains 10 percent, the STS3095 4 percent. A single "centidegrees per second per speed
  unit" factor, which is what the first implementation used, is therefore wrong by a factor
  of four depending on the operating point.
* above the last measured point there is almost nothing left to gain, so asking for more than
  the top of the curve is answered with the maximum setting instead of a fantasy number.

The curve is a property of the servo firmware, identical for every servo of a family on this
robot, and it changes only when a servo is replaced or the supply voltage changes. That is why it
lives here, in the code: versioned, reviewable, no migration, and the measurement is written next
to it. A per-robot deviation would belong in the motor settings, not in a hidden file.
"""

from __future__ import annotations

import bisect
from typing import Sequence

#: Servo ids that carry the STS3095 model; everything else on the bus is a finger servo.
STS3095_SERVO_IDS: frozenset[int] = frozenset({18, 19, 20, 21, 38, 39, 40, 41, 50, 51})

SERVO_FAMILY_STS3095 = "sts3095"
SERVO_FAMILY_FINGER = "finger"

#: Measured (speed setting, cruise rate in centidegrees per second), ascending by setting.
SPEED_CURVE_BY_FAMILY: dict[str, tuple[tuple[int, float], ...]] = {
    SERVO_FAMILY_STS3095: ((200, 2091.8), (700, 4729.3), (2000, 4948.8)),
    SERVO_FAMILY_FINGER: ((200, 1740.5), (700, 3483.3), (2000, 3834.6)),
}

#: The STS speed setting accepts roughly this range.
STS_SPEED_MIN = 50
STS_SPEED_MAX = 3400


def family_for_servo_id(servo_id: int) -> str:
    """Which curve belongs to this servo id."""
    return (
        SERVO_FAMILY_STS3095
        if int(servo_id) in STS3095_SERVO_IDS
        else SERVO_FAMILY_FINGER
    )


def curve_for_family(family: str) -> tuple[tuple[int, float], ...]:
    """The measured curve for a family, falling back to the finger curve."""
    return SPEED_CURVE_BY_FAMILY.get(family, SPEED_CURVE_BY_FAMILY[SERVO_FAMILY_FINGER])


def curve_for_servo_id(servo_id: int) -> tuple[tuple[int, float], ...]:
    return curve_for_family(family_for_servo_id(servo_id))


def rate_for_speed(speed: int, curve: Sequence[tuple[int, float]]) -> float:
    """Forward direction: the cruise rate this speed setting produces, in centideg/s.

    Piecewise linear through the measured points and through the origin, because a setting of
    zero does not move at all. Values outside the measured range are extrapolated along the last
    segment, which is conservative: the real curve flattens further, so the predicted rate is an
    upper bound.
    """
    if not curve:
        return 0.0
    points: list[tuple[int, float]] = [(0, 0.0)] + [
        (int(setting), float(rate)) for setting, rate in curve
    ]
    speed = max(0, int(speed))
    settings = [point[0] for point in points]
    index = bisect.bisect_right(settings, speed) - 1
    if index >= len(points) - 1:
        low_setting, low_rate = points[-2]
        high_setting, high_rate = points[-1]
    else:
        low_setting, low_rate = points[index]
        high_setting, high_rate = points[index + 1]
    if high_setting == low_setting:
        return high_rate
    fraction = (speed - low_setting) / (high_setting - low_setting)
    return low_rate + fraction * (high_rate - low_rate)


def speed_for_rate(
    rate_centideg_per_sec: float, curve: Sequence[tuple[int, float]]
) -> int:
    """Inverse direction: the smallest speed setting that reaches this rate.

    Returns 0 when no movement is asked for, and the maximum setting when the rate is above the
    last measured point - the curve saturates, so a larger number would not make the servo faster.
    """
    if not curve:
        return 0
    wanted = abs(float(rate_centideg_per_sec))
    if wanted <= 0.0:
        return 0
    points: list[tuple[int, float]] = [(0, 0.0)] + [
        (int(setting), float(rate)) for setting, rate in curve
    ]
    if wanted >= points[-1][1]:
        return int(points[-1][0])
    for index in range(len(points) - 1):
        low_setting, low_rate = points[index]
        high_setting, high_rate = points[index + 1]
        if low_rate <= wanted <= high_rate:
            if high_rate == low_rate:
                return int(high_setting)
            fraction = (wanted - low_rate) / (high_rate - low_rate)
            return int(round(low_setting + fraction * (high_setting - low_setting)))
    return int(points[-1][0])


def speed_for_path(
    path_centideg_per_sec: float,
    curve: Sequence[tuple[int, float]],
    *,
    margin: float = 1.2,
    min_speed: int = STS_SPEED_MIN,
    max_speed: int = STS_SPEED_MAX,
) -> int:
    """Speed setting that lets the servo follow a path at this rate.

    ``margin`` above one keeps the servo able to keep up (it must not be the limiting factor);
    the result is clamped into the range the servo accepts, and a path that stands still gets the
    minimum speed rather than zero, so a stationary joint still holds its position.
    """
    needed = abs(float(path_centideg_per_sec)) * max(0.0, float(margin))
    if needed <= 0.0:
        return int(min_speed)
    setting = speed_for_rate(needed, curve)
    return int(max(min_speed, min(max_speed, setting)))
