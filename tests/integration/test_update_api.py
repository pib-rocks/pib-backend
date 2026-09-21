from __future__ import annotations

import json


def _request(channel="release"):
    return {"channel": channel, "force": False, "confirmation": "UPDATE"}


MARKER = '{"schemaVersion":1,"updateCheck":true}'


def _installed_update_dir(tmp_path):
    """A directory as the installer leaves it: present plus its marker."""
    update_dir = tmp_path / "update"
    update_dir.mkdir()
    (update_dir / "service.json").write_text(MARKER, encoding="utf-8")
    return update_dir


def test_start_status_conflict_log_and_cancel(client, tmp_path, monkeypatch):
    update_dir = _installed_update_dir(tmp_path)
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


def test_update_endpoints_report_runner_missing(client, tmp_path, monkeypatch):
    """Docker creates the bind-mount point itself, so a bare directory is not a runner."""
    update_dir = tmp_path / "update"
    update_dir.mkdir()
    monkeypatch.setenv("PIB_UPDATE_DIR", str(update_dir))

    status = client.get("/system/update/status")
    start = client.post("/system/update", json=_request())

    assert status.status_code == 503
    assert status.get_json()["state"] == "runner_missing"
    assert start.status_code == 503
    assert start.get_json()["state"] == "runner_missing"
    assert not (update_dir / "request.json").exists()


def test_revision_endpoint_preserves_unknown_values(client, tmp_path, monkeypatch):
    update_dir = _installed_update_dir(tmp_path)
    monkeypatch.setenv("PIB_UPDATE_DIR", str(update_dir))

    response = client.get("/system/revision")

    assert response.status_code == 200
    assert response.get_json()["repositories"]["pib-backend"]["gitSha"] == "unknown"
    assert response.get_json()["repositories"]["cerebra"]["channel"] == "unknown"


def test_update_check_and_initial_availability(client, tmp_path, monkeypatch):
    update_dir = _installed_update_dir(tmp_path)
    monkeypatch.setenv("PIB_UPDATE_DIR", str(update_dir))

    initial = client.get("/system/update/available")
    queued = client.post("/system/update/check", json={"channel": "develop"})

    assert initial.status_code == 200
    assert initial.get_json()["checkedAt"] is None
    assert (
        initial.get_json()["repositories"]["pib-backend"]["updateAvailable"]
        == "unknown"
    )
    assert initial.get_json()["repositories"]["cerebra"]["target"] == "unknown"
    assert queued.status_code == 202
    assert queued.get_json()["channel"] == "develop"
    assert queued.get_json()["actor"] == "127.0.0.1"
    check_document = json.loads((update_dir / "check.json").read_text())
    assert check_document == queued.get_json()


def test_update_check_conflicts_with_active_update(client, tmp_path, monkeypatch):
    update_dir = _installed_update_dir(tmp_path)
    monkeypatch.setenv("PIB_UPDATE_DIR", str(update_dir))
    assert client.post("/system/update", json=_request()).status_code == 202

    response = client.post("/system/update/check", json={"channel": "release"})

    assert response.status_code == 409
    assert response.get_json()["status"]["classification"] == "queued"
    assert not (update_dir / "check.json").exists()


def test_update_availability_reads_runner_document(client, tmp_path, monkeypatch):
    update_dir = _installed_update_dir(tmp_path)
    monkeypatch.setenv("PIB_UPDATE_DIR", str(update_dir))
    document = {
        "schemaVersion": 1,
        "checkedAt": "2026-09-21T12:00:00+00:00",
        "repositories": {
            "pib-backend": {
                "installed": "a" * 40,
                "target": "b" * 40,
                "updateAvailable": True,
            },
            "cerebra": {
                "installed": "c" * 40,
                "target": "unknown",
                "updateAvailable": "unknown",
                "error": "offline",
            },
        },
    }
    (update_dir / "available.json").write_text(json.dumps(document), encoding="utf-8")

    response = client.get("/system/update/available")

    assert response.status_code == 200
    assert response.get_json() == document


def test_update_check_endpoints_report_unavailable_runner(
    client, tmp_path, monkeypatch
):
    absent = tmp_path / "absent"
    monkeypatch.setenv("PIB_UPDATE_DIR", str(absent))

    assert client.post("/system/update/check", json={}).status_code == 503
    assert client.get("/system/update/available").status_code == 503

    legacy = tmp_path / "legacy"
    legacy.mkdir()
    (legacy / "service.json").write_text('{"schemaVersion":1}', encoding="utf-8")
    monkeypatch.setenv("PIB_UPDATE_DIR", str(legacy))

    queued = client.post("/system/update/check", json={})
    available = client.get("/system/update/available")
    assert queued.status_code == 503
    assert queued.get_json()["state"] == "runner_missing"
    assert available.status_code == 503
    assert available.get_json()["state"] == "runner_missing"
