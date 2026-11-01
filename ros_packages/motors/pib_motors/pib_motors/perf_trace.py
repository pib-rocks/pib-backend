"""Trajectory execution markers for voice-to-motion profiling (PR-2106)."""

from __future__ import annotations


def format_perf_trace_trajectory_start(points: int, elapsed_ms: float) -> str:
    count = max(0, int(points))
    return (
        f"[PERF_TRACE] TRAJECTORY_START points={count} "
        f"elapsed_ms={float(elapsed_ms):.2f}"
    )


def format_perf_trace_trajectory_done(
    duration_sec: float, successful: bool, cancelled: bool
) -> str:
    return (
        f"[PERF_TRACE] TRAJECTORY_DONE duration_sec={float(duration_sec):.3f} "
        f"successful={'true' if successful else 'false'} "
        f"cancelled={'true' if cancelled else 'false'}"
    )


def format_perf_trace_trajectory_leg_motion(
    joint: str,
    leg: int,
    t0_sec: float,
    t1_sec: float,
    path_centideg_per_sec: float,
    sts_speed: int,
    sts_accel: int,
) -> str:
    safe_joint = (joint or "unknown").replace(" ", "_")
    return (
        f"[PERF_TRACE] TRAJECTORY_LEG_MOTION joint={safe_joint} leg={int(leg)} "
        f"t0={float(t0_sec):.3f} t1={float(t1_sec):.3f} "
        f"path_cdps={float(path_centideg_per_sec):.2f} "
        f"sts_speed={int(sts_speed)} sts_accel={int(sts_accel)}"
    )


def format_perf_trace_trajectory_slow_path(
    joint: str,
    min_path_centideg_per_sec: float,
    floor_centideg_per_sec: float,
    floor_rad_per_sec: float,
) -> str:
    safe_joint = (joint or "unknown").replace(" ", "_")
    return (
        f"[PERF_TRACE] TRAJECTORY_SLOW_PATH joint={safe_joint} "
        f"min_path_cdps={float(min_path_centideg_per_sec):.2f} "
        f"floor_cdps={float(floor_centideg_per_sec):.2f} "
        f"floor_rad_s={float(floor_rad_per_sec):.6f}"
    )


def format_perf_trace_joint_state_sweep(
    joint_count: int, elapsed_ms: float, ports_ok: int, ports_total: int
) -> str:
    count = max(0, int(joint_count))
    return (
        f"[PERF_TRACE] JOINT_STATE_SWEEP joints={count} "
        f"elapsed_ms={float(elapsed_ms):.2f} ports_ok={int(ports_ok)}/"
        f"{int(ports_total)}"
    )


def log_perf_trace(node_logger, line: str) -> None:
    try:
        node_logger.info(line)
    except Exception:
        pass
