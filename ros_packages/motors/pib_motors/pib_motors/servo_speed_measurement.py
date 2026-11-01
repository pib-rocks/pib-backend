"""Pure helpers for PR-2153 low-speed calibration (no ROS imports)."""

from __future__ import annotations

import math
import statistics
from typing import Any, Sequence

from pib_motors.servo_speed_curve import CENTIDEG_PER_ENCODER_TICK

MICRO_MOVE_TICKS = (1, 2, 5)
MICRO_MOVE_REPEATS = 10
MAX_STEP_TO_MEDIAN_RATIO = 2.0


def centideg_delta_for_ticks(tick_count: int) -> float:
    """Centidegree delta equivalent to ``tick_count`` encoder ticks."""
    return float(tick_count) * CENTIDEG_PER_ENCODER_TICK


def rad_to_centideg(rad: float) -> float:
    return float(rad) * (18000.0 / math.pi)


def centideg_to_rad(centideg: float) -> float:
    return float(centideg) * (math.pi / 18000.0)


def assert_recording_valid(
    samples: Sequence[tuple[float, float]], *, label: str
) -> str | None:
    """Return an error message when the series is empty or stuck at zero."""
    if not samples:
        return f"{label}: no position samples recorded"
    positions = [pos for _, pos in samples]
    if all(abs(pos) < 1e-9 for pos in positions):
        return (
            f"{label}: all sampled positions are zero — check SSR/joint_states "
            "warm-up or use get_joint_position polling"
        )
    span = max(positions) - min(positions)
    if span < 1e-6:
        return f"{label}: position did not change during the move"
    return None


def cruise_slope_centideg_per_sec(
    samples: Sequence[tuple[float, float]],
    *,
    cruise_start_fraction: float = 0.15,
    cruise_end_fraction: float = 0.85,
) -> float:
    """Cruise rate from wall-clock samples (centideg/s) using the middle of the move."""
    if len(samples) < 2:
        return 0.0
    t0 = samples[0][0]
    t1 = samples[-1][0]
    duration = t1 - t0
    if duration <= 0.0:
        return 0.0
    start_t = t0 + duration * cruise_start_fraction
    end_t = t0 + duration * cruise_end_fraction
    window = [(t, p) for t, p in samples if start_t <= t <= end_t]
    if len(window) < 2:
        window = list(samples)
    wt0, wp0 = window[0]
    wt1, wp1 = window[-1]
    dt = wt1 - wt0
    if dt <= 0.0:
        return 0.0
    return abs(rad_to_centideg(wp1 - wp0)) / dt


def position_step_ratio(samples: Sequence[tuple[float, float]]) -> dict[str, float]:
    """Largest / median absolute step in radians (joint-space samples)."""
    if len(samples) < 2:
        return {
            "median_step_rad": 0.0,
            "largest_step_rad": 0.0,
            "largest_to_median_ratio": 0.0,
        }
    steps = [abs(samples[i][1] - samples[i - 1][1]) for i in range(1, len(samples))]
    ordered = sorted(steps)
    median = ordered[len(ordered) // 2]
    largest = max(steps)
    ratio = largest / median if median > 0.0 else float("inf")
    return {
        "median_step_rad": median,
        "largest_step_rad": largest,
        "largest_to_median_ratio": ratio,
    }


def summarize_cruise_rates(values: Sequence[float]) -> dict[str, float]:
    if not values:
        return {"mean_cdps": 0.0, "stdev_cdps": 0.0, "min_cdps": 0.0, "max_cdps": 0.0}
    if len(values) == 1:
        return {
            "mean_cdps": values[0],
            "stdev_cdps": 0.0,
            "min_cdps": values[0],
            "max_cdps": values[0],
        }
    return {
        "mean_cdps": statistics.mean(values),
        "stdev_cdps": statistics.pstdev(values),
        "min_cdps": min(values),
        "max_cdps": max(values),
    }


def suggest_continuous_floor_cdps(
    setting_stats: dict[int, dict[str, Any]],
    *,
    max_step_ratio: float = MAX_STEP_TO_MEDIAN_RATIO,
) -> tuple[int | None, float | None]:
    """Lowest setting whose runs meet the step-ratio gate; returns (setting, mean cdps)."""
    for setting in sorted(setting_stats):
        stats = setting_stats[setting]
        mean_cdps = stats.get("mean_cdps")
        worst_ratio = stats.get("worst_step_ratio")
        if mean_cdps is None or float(mean_cdps) <= 0.0:
            continue
        if worst_ratio is not None and float(worst_ratio) > max_step_ratio:
            continue
        return int(setting), float(mean_cdps)
    return None, None
