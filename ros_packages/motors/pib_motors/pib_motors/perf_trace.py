"""Trajectory execution markers for voice-to-motion profiling (PR-2106)."""

from __future__ import annotations


def format_perf_trace_trajectory_start(points: int, elapsed_ms: float) -> str:
    count = max(0, int(points))
    return (
        f"[PERF_TRACE] TRAJECTORY_START points={count} "
        f"elapsed_ms={float(elapsed_ms):.2f}"
    )


def log_perf_trace(node_logger, line: str) -> None:
    try:
        node_logger.info(line)
    except Exception:
        pass
