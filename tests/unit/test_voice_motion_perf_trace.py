"""[PERF_TRACE] voice-to-motion chain markers (PR-2106)."""

from __future__ import annotations

import re
import sys
from pathlib import Path
from unittest.mock import MagicMock

import logging

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
VOICE_ROOT = REPO_ROOT / "ros_packages" / "voice_assistant"
MOTORS_ROOT = REPO_ROOT / "ros_packages" / "motors" / "pib_motors"

if str(VOICE_ROOT) not in sys.path:
    sys.path.insert(0, str(VOICE_ROOT))
if str(MOTORS_ROOT) not in sys.path:
    sys.path.insert(0, str(MOTORS_ROOT))

from pib_mcp_server.backend import PibBackend  # noqa: E402
from pib_mcp_server.perf_trace import (  # noqa: E402
    format_perf_trace_motion_cmd,
    log_perf_trace,
)
from pib_motors.perf_trace import format_perf_trace_trajectory_start  # noqa: E402
from voice_assistant.turn_latency import (  # noqa: E402
    format_perf_trace_stt_done,
    log_perf_trace as log_voice_perf_trace,
)


def test_stt_done_line_format():
    line = format_perf_trace_stt_done(
        "chat-42", 812.345, "faster-whisper-small", "local"
    )
    assert line == (
        "[PERF_TRACE] STT_DONE chat=chat-42 elapsed_ms=812.35 "
        "engine=faster-whisper-small path=local"
    )
    assert re.fullmatch(
        r"\[PERF_TRACE\] STT_DONE chat=\S* elapsed_ms=\d+\.\d{2} "
        r"engine=\S+ path=(local|cloud)",
        line,
    )


def test_motion_cmd_line_format():
    line = format_perf_trace_motion_cmd("chat-42", "move_motor", 120.5, True)
    assert line == (
        "[PERF_TRACE] MOTION_CMD chat=chat-42 tool=move_motor "
        "elapsed_ms=120.50 ok=true"
    )
    assert re.fullmatch(
        r"\[PERF_TRACE\] MOTION_CMD chat=\S* tool=\S+ elapsed_ms=\d+\.\d{2} ok=(true|false)",
        line,
    )


def test_trajectory_start_line_format():
    line = format_perf_trace_trajectory_start(3, 4.2)
    assert line == "[PERF_TRACE] TRAJECTORY_START points=3 elapsed_ms=4.20"
    assert re.fullmatch(
        r"\[PERF_TRACE\] TRAJECTORY_START points=\d+ elapsed_ms=\d+\.\d{2}",
        line,
    )


def test_motion_cmd_survives_broken_logger(monkeypatch):
    class BrokenBackend(PibBackend):
        def _ros_service(self, service, service_type, arguments):
            return {"successful": True, "results": []}

    backend = BrokenBackend()
    trace_lines: list[str] = []

    def broken_info(line, *args, **kwargs):
        trace_lines.append(line)
        raise RuntimeError("logger down")

    monkeypatch.setattr("pib_mcp_server.perf_trace.logger.info", broken_info)

    outcome = backend._apply_joint_trajectory_degrees(
        ["head"],
        [{"positions_degrees": [10.0]}],
        trace_tool="move_motor",
    )
    assert outcome["successful"] is True
    assert trace_lines
    assert "MOTION_CMD" in trace_lines[0]


def test_voice_perf_trace_logger_never_raises():
    logger = MagicMock()
    logger.info.side_effect = RuntimeError("broken")
    log_voice_perf_trace(logger, format_perf_trace_stt_done("", 1.0, "x", "local"))


def test_mcp_perf_trace_logger_never_raises(monkeypatch):
    import pib_mcp_server.perf_trace as perf_trace

    def boom(_line):
        raise RuntimeError("broken")

    monkeypatch.setattr(perf_trace.logger, "info", boom)
    log_perf_trace(format_perf_trace_motion_cmd("", "move_motors", 1.0, False))


def test_motion_trace_falls_back_to_stderr_when_no_logger_is_configured(
    capfd, monkeypatch
):
    """The MCP server configures no logging, so logger.info would be dropped.

    Measured on the robot: the tool ran, no MOTION_CMD line appeared. The line has to
    land on stderr (stdout is the MCP protocol channel) when nobody set up a handler.
    """
    from pib_mcp_server import perf_trace

    class Handlerless:
        """Stands in for a logger with no handler anywhere up the chain."""

        def hasHandlers(self):
            return False

        def info(self, line):
            raise AssertionError("a handlerless logger must not be used")

    monkeypatch.setattr(perf_trace, "logger", Handlerless())

    perf_trace.log_perf_trace("[PERF_TRACE] MOTION_CMD chat= probe")

    captured = capfd.readouterr()
    assert "MOTION_CMD chat= probe" in captured.err
    assert captured.out == ""


def test_motion_trace_uses_a_configured_logger(caplog):
    """With a handler in place the line goes through the logger, not around it."""
    from pib_mcp_server import perf_trace

    with caplog.at_level(logging.INFO, logger=perf_trace.__name__):
        perf_trace.log_perf_trace("[PERF_TRACE] MOTION_CMD chat= configured")

    assert any(
        "MOTION_CMD chat= configured" in record.getMessage()
        for record in caplog.records
    )


def test_motion_trace_survives_a_broken_logger(monkeypatch):
    """A logger that raises must not break the tool that emits the line."""
    from pib_mcp_server import perf_trace

    class Exploding:
        def hasHandlers(self):
            raise RuntimeError("logger is on fire")

    monkeypatch.setattr(perf_trace, "logger", Exploding())
    perf_trace.log_perf_trace("[PERF_TRACE] MOTION_CMD chat= broken")
