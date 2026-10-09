"""Operator-journey contract. These assertions describe the real API, not a fixture invented by the UI."""

from __future__ import annotations

import json
from datetime import datetime, timedelta, timezone

def _request(channel="release"):
    return {"channel": channel, "force": False, "confirmation": "UPDATE"}


def _installed_update_dir(tmp_path):
    update_dir = tmp_path / "update"
    update_dir.mkdir()
    (update_dir / "service.json").write_text(
        '{"schemaVersion":1,"updateCheck":true}', encoding="utf-8"
    )
    return update_dir


def test_accepted_job_has_no_state_and_status_is_separate(client, tmp_path, monkeypatch):
    update_dir = _installed_update_dir(tmp_path)
    monkeypatch.setenv("PIB_UPDATE_DIR", str(update_dir))

    started = client.post("/system/update", json=_request())

    assert started.status_code == 202
    body = started.get_json()
    assert "state" not in body["job"]
    assert "classification" not in body["job"]
    assert body["status"]["classification"] == "queued"
    assert body["status"]["jobId"] == body["job"]["jobId"]


def test_pending_check_does_not_answer_with_the_previous_document(
    client, tmp_path, monkeypatch
):
    update_dir = _installed_update_dir(tmp_path)
    monkeypatch.setenv("PIB_UPDATE_DIR", str(update_dir))
    previous = {
        "schemaVersion": 1,
        "checkId": "00000000-0000-4000-8000-000000000001",
        "state": "completed",
        "channel": "release",
        "checkedAt": "2020-01-01T00:00:00+00:00",
        "repositories": {
            "pib-backend": {
                "installed": "a" * 40,
                "target": "b" * 40,
                "updateAvailable": True,
            }
        },
    }
    (update_dir / "available.json").write_text(json.dumps(previous), encoding="utf-8")

    queued = client.post("/system/update/check", json={"channel": "release"})
    available = client.get("/system/update/available")

    assert queued.status_code == 202
    check_id = queued.get_json()["checkId"]
    body = available.get_json()
    assert body["state"] == "pending"
    assert body["checkId"] == check_id
    assert body["checkId"] != previous["checkId"]
    assert "repositories" not in body
    assert body["previous"]["checkId"] == previous["checkId"]
    assert body["previous"]["repositories"]["pib-backend"]["target"] == "b" * 40


def test_nonterminal_status_without_a_live_request_is_stale_and_not_blocking(
    client, tmp_path, monkeypatch
):
    update_dir = _installed_update_dir(tmp_path)
    monkeypatch.setenv("PIB_UPDATE_DIR", str(update_dir))
    (update_dir / "status.json").write_text(
        json.dumps(
            {
                "schemaVersion": 1,
                "jobId": "stale-job",
                "channel": "release",
                "state": "building",
                "message": "Building",
                "updatedAt": "2020-01-01T00:00:00+00:00",
            }
        ),
        encoding="utf-8",
    )

    observed = client.get("/system/update/status")
    started = client.post("/system/update", json=_request())

    assert observed.status_code == 200
    assert observed.get_json()["classification"] == "stale"
    assert observed.get_json()["jobId"] == "stale-job"
    assert "block" in observed.get_json()["staleReason"].lower() or observed.get_json()[
        "staleReason"
    ]
    assert started.status_code == 202
    assert started.get_json()["job"]["jobId"] != "stale-job"


def test_queued_request_past_the_start_deadline_is_recoverable(
    client, tmp_path, monkeypatch
):
    update_dir = _installed_update_dir(tmp_path)
    monkeypatch.setenv("PIB_UPDATE_DIR", str(update_dir))
    requested_at = (datetime.now(timezone.utc) - timedelta(hours=1)).isoformat()
    (update_dir / "request.json").write_text(
        json.dumps(
            {
                "schemaVersion": 1,
                "jobId": "never-started",
                "requestedAt": requested_at,
                "actor": "127.0.0.1",
                "channel": "release",
                "force": False,
                "confirmation": "UPDATE",
            }
        ),
        encoding="utf-8",
    )

    observed = client.get("/system/update/status")
    started = client.post("/system/update", json=_request())

    assert observed.get_json()["classification"] == "stale"
    assert observed.get_json()["state"] == "queued"
    assert "start" in observed.get_json()["staleReason"].lower()
    assert started.status_code == 202
