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

PR-2153 (low speed, measured on the pib5advanced)
-----------------------------------------------
Measured with the wall clock, not with the position series: one move of 0.5 rad (turn_head_motor)
or 0.3 rad (index_right_stretch), time from command to arrival at the target, three runs per
setting, speed = distance / time. The position series is the weaker method below setting 100 -
at setting 200 the joint covers 0.07 rad in ten seconds, which is 0.4 centidegrees per sample at
30 Hz, and the slope turns into noise (a first run reported the curve upside down because of it).

Findings:

- The setting is monotone upwards and saturates around 700.
- **There is a floor: setting 1 and setting 50 both give ~447 cdg/s (STS3095) and ~426 cdg/s
  (finger).** Below about 50 the setting changes nothing, the servo runs on its mechanical
  minimum. That floor is the slowest path a trajectory can follow continuously.
- Setting 200 is 1736 cdg/s (STS3095), not the 2091.8 that the position-series method reported:
  the older numbers are about 17 % too fast, which is exactly the direction that made slow paths
  step.

Still unmeasured: setting 0 (must not be used, see below), the micro-move reliability, and the
finger point at 2000 - the finger run timed out below setting 100 because the joint sits at its
configured lower limit, so those rows are left out instead of guessed.

Speed setting 0 must not appear in motor or trajectory commands until
``STS_SPEED_SETTING_ZERO_MEASURED_BEHAVIOR`` is filled in: on several STS firmware generations 0
means maximum speed instead of stop.

Trajectories slower than ``MIN_CONTINUOUS_PATH_SPEED_CDPS_BY_FAMILY`` cannot be tracked
continuously while ``STS_SPEED_MIN`` clamps the setting above the path rate (the servo finishes
each 10 ms tick early and motion steps). ``SLOW_PATH_BELOW_FLOOR_POLICY`` is ``warn``: emit
``[PERF_TRACE] TRAJECTORY_SLOW_PATH`` and execute anyway. Rejecting would break slow Blockly
motions without removing the physical limit; the trace makes the compromise visible in logs.
"""

from __future__ import annotations

import bisect
import math
from typing import Sequence

#: Servo ids that carry the STS3095 model; everything else on the bus is a finger servo.
STS3095_SERVO_IDS: frozenset[int] = frozenset({18, 19, 20, 21, 38, 39, 40, 41, 50, 51})

SERVO_FAMILY_STS3095 = "sts3095"
SERVO_FAMILY_FINGER = "finger"

#: Measured (speed setting, cruise rate in centidegrees per second), ascending by setting.
SPEED_CURVE_BY_FAMILY: dict[str, tuple[tuple[int, float], ...]] = {
    SERVO_FAMILY_STS3095: ((700, 4436.5), (2000, 4565.9)),
    SERVO_FAMILY_FINGER: ((700, 3649.0),),
}

#: The STS speed setting accepts roughly this range.
STS_SPEED_MIN = 50
STS_SPEED_MAX = 3400

#: STS encoder quantisation: 4096 ticks per revolution in driver units.
ENCODER_TICKS_PER_REVOLUTION = 4096
CENTIDEG_PER_REVOLUTION = 36000.0
CENTIDEG_PER_ENCODER_TICK = CENTIDEG_PER_REVOLUTION / ENCODER_TICKS_PER_REVOLUTION

#: Settings measured by tools/measure_servo_speed_floor.py (PR-2153).
LOW_SPEED_CALIBRATION_SETTINGS: tuple[int, ...] = (1, 5, 10, 20, 50, 100, 200)

#: Cruise centideg/s at each setting; replace None after measurement (one float per row).
LOW_SPEED_CURVE_BY_FAMILY: dict[str, tuple[tuple[int, float | None], ...]] = {
    SERVO_FAMILY_STS3095: (
        (1, 448.7),
        (50, 446.4),
        (100, 887.9),
        (200, 1736.3),
    ),
    SERVO_FAMILY_FINGER: (
        (1, 436.3),
        (10, 425.8),
        (100, 855.2),
        (200, 1629.2),
    ),
}

#: Slowest path rate (centideg/s) that still looks continuous on hardware; None until measured.
MIN_CONTINUOUS_PATH_SPEED_CDPS_BY_FAMILY: dict[str, float | None] = {
    SERVO_FAMILY_STS3095: 447.0,
    SERVO_FAMILY_FINGER: 426.0,
}

#: Smallest encoder tick delta that arrived reliably in micro-move tests; None until measured.
MIN_RELIABLE_MOVE_TICKS_BY_FAMILY: dict[str, int | None] = {
    SERVO_FAMILY_STS3095: None,
    SERVO_FAMILY_FINGER: None,
}

#: Do not command setting 0 until this is filled (e.g. ``"max_speed"`` or ``"stop"``).
STS_SPEED_SETTING_ZERO_FORBIDDEN = True
STS_SPEED_SETTING_ZERO_MEASURED_BEHAVIOR: str | None = None

SLOW_PATH_POLICY_WARN = "warn"
SLOW_PATH_POLICY_REJECT = "reject"
SLOW_PATH_BELOW_FLOOR_POLICY = SLOW_PATH_POLICY_WARN


def rad_per_sec_from_centideg_per_sec(centideg_per_sec: float) -> float:
    """Convert a centidegree-per-second rate to rad/s."""
    return float(centideg_per_sec) * (math.pi / 18000.0)


def centideg_per_sec_from_rad_per_sec(rad_per_sec: float) -> float:
    """Convert rad/s to centidegrees per second."""
    return float(rad_per_sec) * (18000.0 / math.pi)


def continuous_path_floor_centideg_per_sec(family: str) -> float | None:
    """Minimum path speed for continuous tracking, or None until PR-2153 measurement."""
    value = MIN_CONTINUOUS_PATH_SPEED_CDPS_BY_FAMILY.get(family)
    if value is None:
        return None
    return float(value)


def merged_measured_curve(family: str) -> tuple[tuple[int, float], ...]:
    """Combine pending low-speed rows with the PR-2142 curve, skipping unmeasured points."""
    by_setting: dict[int, float] = {}
    for setting, rate in LOW_SPEED_CURVE_BY_FAMILY.get(family, ()):
        if rate is not None:
            by_setting[int(setting)] = float(rate)
    for setting, rate in SPEED_CURVE_BY_FAMILY.get(family, ()):
        by_setting.setdefault(int(setting), float(rate))
    return tuple(sorted(by_setting.items(), key=lambda item: item[0]))


def family_for_servo_id(servo_id: int) -> str:
    """Which curve belongs to this servo id."""
    return (
        SERVO_FAMILY_STS3095
        if int(servo_id) in STS3095_SERVO_IDS
        else SERVO_FAMILY_FINGER
    )


def curve_for_family(family: str) -> tuple[tuple[int, float], ...]:
    """The measured curve for a family, falling back to the finger curve."""
    merged = merged_measured_curve(family)
    if merged:
        return merged
    return SPEED_CURVE_BY_FAMILY.get(family, SPEED_CURVE_BY_FAMILY[SERVO_FAMILY_FINGER])


def curve_for_servo_id(servo_id: int) -> tuple[tuple[int, float], ...]:
    return curve_for_family(family_for_servo_id(servo_id))


def rate_for_speed(speed: int, curve: Sequence[tuple[int, float]]) -> float:
    """Forward direction: the cruise rate this speed setting produces, in centideg/s.

    Piecewise linear through the measured points and through the origin when no point exists at
    zero (below 200 that segment is extrapolated, not measured — see PR-2153). Values outside
    the measured range are extrapolated along the last
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
