"""VA-ERR-1 (PR-2274): trace when pib.Cloud live session ends abnormally."""

from __future__ import annotations

import json
import sys
from pathlib import Path
from unittest.mock import MagicMock

import pytest

_UNIT = Path(__file__).resolve().parent
if str(_UNIT) not in sys.path:
    sys.path.insert(0, str(_UNIT))

from test_assistant_voice_lifecycle import assistant_node  # noqa: E402


def _live_session_ended_events(publisher) -> list[dict]:
    events = []
    for call in publisher.publish.call_args_list:
        raw = call[0][0].data
        event = json.loads(raw)
        if event.get("stage") == "live_session_ended":
            events.append(event)
    return events


def test_abnormal_cloud_live_session_end_publishes_live_session_ended_trace(
    assistant_node,
):
    node = assistant_node
    node._turn_trace_publisher = MagicMock()
    node.state.chat_id = "chat-quota"
    node.state.turned_on = False
    node._dialog_active = False

    node._on_cloud_live_session_end("daily_live_quota_exhausted")

    events = _live_session_ended_events(node._turn_trace_publisher)
    assert len(events) == 1
    event = events[0]
    assert event["stage"] == "live_session_ended"
    assert event["source"] == "pib-cloud"
    assert event["chat_id"] == "chat-quota"
    assert event["detail"] == {"reason": "daily_live_quota_exhausted"}
    assert isinstance(event["ts_ms"], (int, float))


def test_normal_cloud_live_session_end_closed_does_not_publish_error_trace(
    assistant_node,
):
    node = assistant_node
    node._turn_trace_publisher = MagicMock()
    node.state.turned_on = False
    node._dialog_active = False

    node._on_cloud_live_session_end("closed")

    assert _live_session_ended_events(node._turn_trace_publisher) == []
