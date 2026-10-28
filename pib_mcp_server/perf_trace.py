"""Greppable motion-tool latency lines for voice-to-motion profiling (PR-2106)."""

from __future__ import annotations

import logging
import os
import sys

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
    """Emit one perf line so it actually reaches the container log.

    The MCP server runs as a stdio child of the daemon and configures no logging.
    ``logger.info`` then falls through to logging's lastResort handler, which drops
    everything below WARNING - the line vanished and the motion trace was silently
    dead (measured on the robot: no MOTION_CMD line although the tool ran). Keep the
    logger when someone configured one; otherwise write to stderr, never stdout -
    stdout is the MCP protocol channel.
    """
    try:
        if logger.hasHandlers():
            logger.info(line)
        else:
            print(line, file=sys.stderr, flush=True)
    except Exception:
        pass
