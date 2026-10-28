"""Greppable motion-tool latency lines for voice-to-motion profiling (PR-2106)."""

from __future__ import annotations

import logging
import os
import sys
import time
from pathlib import Path

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


def perf_trace_log_path() -> Path:
    """Where the motion trace is appended.

    The MCP server runs as a child of the daemon with its stderr redirected by the
    host, so a trace line that only goes to stderr may land nowhere the operator can
    see (measured on the robot: mcp-stderr.log stayed empty). A host-mounted file is
    the one sink that cannot be redirected away by the spawn plumbing.
    """
    override = (os.environ.get("PIB_PERF_TRACE_LOG") or "").strip()
    if override:
        return Path(override)
    # HERMES_HOME first: inside the daemon container HOME is /root, so Path.home()
    # wrote the line to /root/.hermes/logs - present in the container, invisible to the
    # host. Measured on the robot: 178 bytes in the wrong place, nothing in the mount.
    hermes_home = (os.environ.get("HERMES_HOME") or "").strip()
    base = Path(hermes_home) if hermes_home else Path.home() / ".hermes"
    return base / "logs" / "pib-perf-trace.log"


def _append_trace_line(line: str) -> None:
    path = perf_trace_log_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "a", encoding="utf-8", errors="replace") as handle:
        handle.write(f"{time.time():.3f} {line}\n")


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
    # The sink that survives any stderr redirection in the spawn plumbing.
    try:
        _append_trace_line(line)
    except Exception:
        pass
