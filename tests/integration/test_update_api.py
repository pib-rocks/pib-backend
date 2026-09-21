from __future__ import annotations

import json


def _request(channel="release"):
    return {"channel": channel, "force": False, "confirmation": "UPDATE"}


def test_start_status_conflict_log_and_cancel(client, tmp_path, monkeypatch):
    update_dir = tmp_path / "update"
    update_dir.mkdir()
    (update_dir / "update.log").write_text("first\nsecond\n", encoding="utf-8")
    monkeypatch.setenv("PIB_UPDATE_DIR", str(update_dir))

    started = client.post("/system/update", json=_request())
    assert started.status_code == 202
    job = started.get_json()["job"]
    assert job["channel"] == "release"
    assert job["actor"] == "127.0.0.1"
    assert started.get_json()["programRunningSignal"] == "unavailable"
    request_document = json.loads((update_dir / "request.json").read_text())
    assert request_document["jobId"] == job["jobId"]

    status = client.get("/system/update/status")
    assert status.status_code == 200
    assert status.get_json()["classification"] == "queued"

    conflict = client.post("/system/update", json=_request())
    assert conflict.status_code == 409
    assert conflict.get_json()["status"]["classification"] == "queued"

    log = client.get("/system/update/log?offset=6")
    assert log.status_code == 200
    assert log.get_json()["content"] == "second\n"

    cancelled = client.post("/system/update/cancel")
    assert cancelled.status_code == 202
    assert cancelled.get_json()["status"]["cancelRequested"] is True
    assert (update_dir / "cancel.json").is_file()


def test_update_rejects_bad_channel_and_missing_confirmation(
    client, tmp_path, monkeypatch
):
    update_dir = tmp_path / "update"
    update_dir.mkdir()
    monkeypatch.setenv("PIB_UPDATE_DIR", str(update_dir))

    bad_channel = client.post("/system/update", json=_request("nightly"))
    missing_confirmation = client.post(
        "/system/update", json={"channel": "release", "force": False}
    )

    assert bad_channel.status_code == 400
    assert "channel" in bad_channel.get_json()["error"]
    assert missing_confirmation.status_code == 400
    assert "confirmation" in missing_confirmation.get_json()["error"]


def test_update_endpoints_report_not_installed(client, tmp_path, monkeypatch):
    monkeypatch.setenv("PIB_UPDATE_DIR", str(tmp_path / "absent"))

    status = client.get("/system/update/status")
    start = client.post("/system/update", json=_request())

    assert status.status_code == 503
    assert status.get_json()["state"] == "not_installed"
    assert start.status_code == 503
    assert start.get_json()["state"] == "not_installed"


def test_revision_endpoint_preserves_unknown_values(client, tmp_path, monkeypatch):
    update_dir = tmp_path / "update"
    update_dir.mkdir()
    monkeypatch.setenv("PIB_UPDATE_DIR", str(update_dir))

    response = client.get("/system/revision")

    assert response.status_code == 200
    assert response.get_json()["repositories"]["pib-backend"]["gitSha"] == "unknown"
    assert response.get_json()["repositories"]["cerebra"]["channel"] == "unknown"
