"""ROS edge (radians) ↔ internal motor control (centidegrees).

Centidegrees are hundredths of a degree (9000 = 90°). Driver layers map
centidegrees to hardware ticks separately (see ``feetech_actuator``).

Rounding: ``rad_to_centideg`` uses ``round()`` to the nearest integer
centidegree (Python's round-to-even for ties). ``centideg_to_rad`` is the
exact inverse for integer centidegrees derived that way.
"""

from __future__ import annotations

import math

CENTIDEG_PER_RADIAN = 18000.0 / math.pi


def rad_to_centideg(value: float) -> int:
    """Convert a joint position in radians to integer centidegrees."""
    if not math.isfinite(value):
        raise ValueError(f"joint position must be finite, got {value}")
    return int(round(math.degrees(value) * 100.0))


def centideg_to_rad(value: int | float) -> float:
    """Convert centidegrees to radians for ROS ``trajectory_msgs`` / ``sensor_msgs``."""
    return math.radians(float(value) / 100.0)
