"""PR-2122: voice-assistant Docker health probe."""

from __future__ import annotations

from unittest.mock import patch

import pytest

from service import voice_assistant_health_service as health


@pytest.fixture
def client(app):
    return app.test_client()


def test_voice_health_ok_when_container_probe_is_healthy(monkeypatch):
    monkeypatch.setattr(
        health,
        "_list_containers",
        lambda: [
            {
                "name": "multirepo-ros-voice-assistant-1",
                "state": "running",
                "statusText": "Up 2 hours (healthy)",
            }
        ],
    )
    monkeypatch.setattr(
        health,
        "_inspect_container",
        lambda _name: {
            "State": {"Health": {"Status": "healthy", "Log": [{"Output": ""}]}}
        },
    )

    result = health.voice_assistant_status()

    assert result["state"] == health.STATE_OK
    assert result["node"]["state"] == health.STATE_HEALTHY
    assert result["providerKey"]["state"] == health.STATE_OK
    assert result["containerHealth"] == health.STATE_HEALTHY


def test_voice_health_missing_key_is_attention(monkeypatch):
    """Load-bearing: a missing key is the actionable fault this probe exists for.

    The .112 state was exactly this - container up, node up, no key - and nothing said so.
    """
    probe_line = (
        "reason=no_key provider=gemini "
        "detail=No keys are available for provider gemini."
    )
    monkeypatch.setattr(
        health,
        "_list_containers",
        lambda: [
            {
                "name": "multirepo-ros-voice-assistant-1",
                "state": "running",
                "statusText": "Up 2 hours (unhealthy)",
            }
        ],
    )
    monkeypatch.setattr(
        health,
        "_inspect_container",
        lambda _name: {
            "State": {
                "Health": {
                    "Status": "unhealthy",
                    "Log": [{"ExitCode": 1, "Output": probe_line + "\n"}],
                }
            }
        },
    )

    result = health.voice_assistant_status()

    assert result["state"] == health.STATE_ATTENTION
    assert result["node"]["state"] == health.STATE_HEALTHY
    assert result["providerKey"]["state"] == health.STATE_MISSING
    assert result["providerKey"]["reason"] == health.REASON_NO_KEY
    assert result["provider"] == "gemini"
    assert result["containerHealth"] == health.STATE_UNHEALTHY


def test_voice_health_without_container_is_unknown(monkeypatch):
    monkeypatch.setattr(health, "_list_containers", lambda: [])

    result = health.voice_assistant_status()

    assert result["state"] == health.STATE_UNKNOWN
    assert result["reason"] == health.REASON_CONTAINER_ABSENT
    assert result["node"]["state"] == health.STATE_UNKNOWN
    assert result["providerKey"]["state"] == health.STATE_UNKNOWN


def test_get_voice_assistant_health_route(client, monkeypatch):
    monkeypatch.setattr(
        health,
        "voice_assistant_status",
        lambda: {
            "state": health.STATE_HEALTHY,
            "node": {"state": health.STATE_HEALTHY},
        },
    )

    response = client.get("/system/voice-assistant-health")

    assert response.status_code == 200
    assert response.get_json()["state"] == health.STATE_HEALTHY


def test_aggregate_state_only_speaks_the_ui_vocabulary(monkeypatch):
    """Load-bearing for the UI contract: the chip parser accepts ok/attention/unknown only.

    Any other value silently degrades to neutral in the browser, which would hide a fault.
    """
    ui_states = {health.STATE_OK, health.STATE_ATTENTION, health.STATE_UNKNOWN}
    probes = [
        ("healthy", ""),
        ("unhealthy", "reason=no_key provider=gemini detail=No keys are available."),
        ("unhealthy", "reason=chat_node detail=missing nodes: /chat"),
        ("unhealthy", "reason=key_check detail=boom"),
        ("unhealthy", "reason=something_else detail=boom"),
        ("unknown", ""),
        ("starting", ""),
    ]
    for docker_health, output in probes:
        monkeypatch.setattr(
            health,
            "_list_containers",
            lambda: [
                {
                    "name": "multirepo-ros-voice-assistant-1",
                    "state": "running",
                    "statusText": "Up 2 hours",
                }
            ],
        )
        monkeypatch.setattr(
            health,
            "_inspect_container",
            lambda _name, d=docker_health, o=output: {
                "State": {"Health": {"Status": d, "Log": [{"Output": o}]}}
            },
        )
        result = health.voice_assistant_status()
        assert result["state"] in ui_states, (docker_health, output, result["state"])
