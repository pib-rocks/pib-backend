"""Cached robot body context for Direct system prompts."""

import sys
import time
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
VOICE_ROOT = REPO_ROOT / "ros_packages" / "voice_assistant"
API_CLIENT_DIR = REPO_ROOT / "pib_api" / "client"
for path in (str(VOICE_ROOT), str(API_CLIENT_DIR)):
    if path not in sys.path:
        sys.path.insert(0, path)

from voice_assistant import robot_context as robot_context_mod  # noqa: E402


@pytest.fixture(autouse=True)
def reset_robot_context_cache():
    robot_context_mod.invalidate()
    robot_context_mod._fetch_failed_logged = False
    yield
    robot_context_mod.invalidate()
    robot_context_mod._fetch_failed_logged = False


def test_failure_returns_empty_without_raising(monkeypatch):
    def fail():
        raise RuntimeError("offline")

    monkeypatch.setattr(robot_context_mod.motion_model_client, "get_motion_model", fail)
    assert robot_context_mod.get_robot_context() == ""


def test_second_call_inside_ttl_does_not_fetch_again(monkeypatch):
    calls = 0

    def succeed():
        nonlocal calls
        calls += 1
        return True, {"prompt": "Body (centidegrees: 1 = 0.01 degree)."}

    monkeypatch.setattr(
        robot_context_mod.motion_model_client, "get_motion_model", succeed
    )
    monkeypatch.setattr(robot_context_mod, "ROBOT_CONTEXT_TTL_SEC", 30)

    assert robot_context_mod.get_robot_context().startswith("Body")
    assert robot_context_mod.get_robot_context().startswith("Body")
    assert calls == 1

    robot_context_mod._cached_at = time.monotonic() - 31
    assert robot_context_mod.get_robot_context().startswith("Body")
    assert calls == 2
