"""Greppable motion-tool latency lines for voice-to-motion profiling (PR-2106)."""

from __future__ import annotations

import logging
import os

logger = logging.getLogger(__name__)


def perf_trace_chat_id() -> str:
    """Active chat id when the Hermes turn sets ``PIB_PERF_TRACE_CHAT_ID``."""
    return (os.environ.get("PIB_PERF_TRACE_CHAT_ID") or "").strip()


def format_perf_trace_motion_cmd(
    chat_id: str, tool: str, elapsed_ms: float, ok: bool
) -> str:
    safe_id = chat_id or ""
    safe_tool = (tool or "unknown").replace(" ", "_")
    return (
        f"[PERF_TRACE] MOTION_CMD chat={safe_id} tool={safe_tool} "
        f"elapsed_ms={float(elapsed_ms):.2f} ok={'true' if ok else 'false'}"
    )


def log_perf_trace(line: str) -> None:
    try:
        logger.info(line)
    except Exception:
        pass
