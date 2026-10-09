from __future__ import annotations

from datetime import datetime, timedelta, timezone
import json

import pytest

from service import revision_service, update_service


def test_request_validation_and_building():
    request = update_service.build_request(
        channel="develop",
        force=True,
        confirmation="UPDATE",
        actor="192.0.2.1",
        job_id="job-1",
        requested_at="2026-09-21T02:00:00+00:00",
    )

    assert request == {
        "schemaVersion": 1,
        "jobId": "job-1",
        "requestedAt": "2026-09-21T02:00:00+00:00",
        "actor": "192.0.2.1",
        "channel": "develop",
        "force": True,
        "confirmation": "UPDATE",
    }


@pytest.mark.parametrize(
    ("overrides", "message"),
    [
        ({"channel": "nightly"}, "channel"),
        ({"force": "yes"}, "boolean"),
        ({"confirmation": "update"}, "confirmation"),
        ({"actor": ""}, "actor"),
    ],
)
def test_request_validation_rejects_bad_fields(overrides, message):
    fields = {
        "channel": "release",
        "force": False,
        "confirmation": "UPDATE",
        "actor": "test",
    }
    fields.update(overrides)

    with pytest.raises(update_service.UpdateValidationError, match=message):
        update_service.build_request(**fields)


@pytest.mark.parametrize(
    ("status", "pending", "classification"),
    [
        (None, False, "idle"),
        (None, True, "queued"),
        ({"state": "building"}, False, "running"),
        ({"state": "done"}, False, "succeeded"),
        ({"state": "failed"}, False, "failed"),
        ({"state": "rolled_back"}, False, "rolled_back"),
        ({"state": "done"}, True, "queued"),
        ({"state": "invented"}, False, "unknown"),
    ],
)
def test_state_classification(status, pending, classification):
    assert update_service.classify_state(status, pending) == classification


def test_update_availability_does_not_guess_unknown_values():
    assert update_service.evaluate_update_available(
        {"pib-backend": "abc", "cerebra": "unknown"},
        {"pib-backend": "def", "cerebra": "123"},
    ) == {"cerebra": "unknown", "pib-backend": True}


def test_atomic_request_write_replaces_complete_json(tmp_path, monkeypatch):
    destination = tmp_path / "request.json"
    real_replace = update_service.os.replace
    replacements = []

    def recording_replace(source, target):
        replacements.append((source, target))
        real_replace(source, target)

    monkeypatch.setattr(update_service.os, "replace", recording_replace)
    update_service.atomic_write_json(destination, {"jobId": "job-1"})

    assert json.loads(destination.read_text(encoding="utf-8")) == {"jobId": "job-1"}
    assert len(replacements) == 1
    assert replacements[0][1] == destination
    # No partially written temporary file may survive, and the file must be
    # group-readable for the host runner (User=pib) that reads requests written
    # by the root container.
    assert not [
        entry for entry in tmp_path.iterdir() if entry.name.startswith(".request.json.")
    ]
    assert destination.stat().st_mode & 0o060 == 0o060


def test_log_offsets_are_byte_offsets(tmp_path):
    (tmp_path / "update.log").write_bytes("one\n€\n".encode())

    first = update_service.read_log(0, tmp_path)
    second = update_service.read_log(4, tmp_path)

    assert first["nextOffset"] == len("one\n€\n".encode())
    assert second["content"] == "€\n"
    assert second["offset"] == 4


def test_missing_directory_reports_not_installed(tmp_path):
    status = update_service.get_status(tmp_path / "absent")

    assert status["state"] == "not_installed"
    assert status["classification"] == "not_installed"


def test_directory_without_marker_reports_runner_missing(tmp_path):
    """Docker creates the bind-mount point on its own - a bare directory is not a runner."""
    directory = tmp_path / "update"
    directory.mkdir()

    status = update_service.get_status(directory)

    assert status["state"] == "runner_missing"
    assert status["classification"] == "runner_missing"
    assert update_service.SERVICE_MARKER_NAME in status["error"]
    assert update_service.has_service_marker(directory) is False


def test_enqueue_refuses_while_the_runner_marker_is_missing(tmp_path):
    directory = tmp_path / "update"
    directory.mkdir()

    with pytest.raises(update_service.UpdateNotInstalledError) as error:
        update_service.enqueue_update(
            {"jobId": "job-1", "channel": "develop", "confirmation": "UPDATE"},
            directory,
        )

    assert error.value.state == "runner_missing"
    assert not (directory / "request.json").exists()


def test_enqueue_accepts_a_directory_with_the_installer_marker(tmp_path):
    directory = tmp_path / "update"
    directory.mkdir()
    (directory / update_service.SERVICE_MARKER_NAME).write_text(
        '{"schemaVersion":1}', encoding="utf-8"
    )
    document = update_service.build_request(
        channel="develop",
        force=False,
        confirmation="UPDATE",
        actor="127.0.0.1",
        job_id="job-1",
        requested_at=datetime.now(timezone.utc).isoformat(),
    )

    status = update_service.enqueue_update(document, directory)

    assert status["classification"] == "queued"
    assert (directory / "request.json").is_file()
    assert update_service.get_status(directory)["state"] == "queued"


def test_revision_parser_uses_unknown_instead_of_guesses():
    assert revision_service.parse_revision("cerebra", {"gitSha": " abc "}) == {
        "repository": "cerebra",
        "gitSha": "abc",
        "buildTime": "unknown",
        "channel": "unknown",
    }


def test_installed_revisions_reads_runner_files(tmp_path, monkeypatch):
    (tmp_path / "pib-backend.revision.json").write_text(
        '{"gitSha":"abc","buildTime":"now","channel":"release"}',
        encoding="utf-8",
    )
    monkeypatch.setattr(revision_service, "read_app_version", lambda: "v1")

    revisions = revision_service.installed_revisions(tmp_path)

    assert revisions["imageVersion"] == "v1"
    assert revisions["repositories"]["pib-backend"]["gitSha"] == "abc"
    assert revisions["repositories"]["cerebra"]["gitSha"] == "unknown"


def test_program_running_hook_explicitly_has_no_signal():
    assert update_service.program_running_signal() is None


def _marker(directory, **extra):
    document = {"schemaVersion": 1, "updateCheck": True, "runner": "/host/setup/update_runner.sh"}
    document.update(extra)
    (directory / "service.json").write_text(json.dumps(document), encoding="utf-8")


def test_old_queued_request_is_stale_until_a_new_one_is_accepted(tmp_path):
    directory = tmp_path / "update"
    directory.mkdir()
    _marker(directory, units=list(update_service.HOST_UPDATE_UNITS))
    requested_at = (datetime.now(timezone.utc) - timedelta(hours=1)).isoformat()
    update_service.atomic_write_json(
        directory / "request.json",
        {
            "schemaVersion": 1,
            "jobId": "old-job",
            "requestedAt": requested_at,
            "channel": "release",
        },
    )

    stale = update_service.get_status(directory)
    assert stale["classification"] == "stale"
    assert stale["blocksNewUpdate"] is False
    assert update_service.is_active(stale, True) is False

    fresh = update_service.build_request(
        channel="release",
        force=False,
        confirmation="UPDATE",
        actor="127.0.0.1",
    )
    accepted = update_service.enqueue_update(fresh, directory)
    assert accepted["classification"] == "queued"
    assert accepted["interruptedJob"]["jobId"] == "old-job"


def test_active_job_with_a_fresh_heartbeat_stays_running(tmp_path):
    directory = tmp_path / "update"
    directory.mkdir()
    _marker(directory)
    now = datetime.now(timezone.utc)
    update_service.atomic_write_json(
        directory / "request.json",
        {"jobId": "job-live", "requestedAt": now.isoformat(), "channel": "release"},
    )
    update_service.atomic_write_json(
        directory / "status.json",
        {"jobId": "job-live", "state": "building", "channel": "release"},
    )
    update_service.atomic_write_json(
        directory / "executor.json",
        {"jobId": "job-live", "updatedAt": now.isoformat(), "pid": 1},
    )

    status = update_service.get_status(directory, now=now)
    assert status["classification"] == "running"
    assert status["blocksNewUpdate"] is True


def test_stopped_heartbeat_does_not_block_forever(tmp_path):
    directory = tmp_path / "update"
    directory.mkdir()
    _marker(directory)
    now = datetime.now(timezone.utc)
    update_service.atomic_write_json(
        directory / "request.json",
        {"jobId": "job-dead", "requestedAt": now.isoformat(), "channel": "develop"},
    )
    update_service.atomic_write_json(
        directory / "status.json",
        {"jobId": "job-dead", "state": "building", "channel": "develop"},
    )
    update_service.atomic_write_json(
        directory / "executor.json",
        {
            "jobId": "job-dead",
            "updatedAt": (now - timedelta(minutes=10)).isoformat(),
            "pid": 1,
        },
    )

    status = update_service.get_status(directory, now=now)
    assert status["classification"] == "stale"
    assert "heartbeat" in status["staleReason"].lower()


def test_readiness_requires_attested_units_and_does_not_treat_marker_as_liveness(tmp_path):
    directory = tmp_path / "update"
    directory.mkdir()
    (directory / "service.json").write_text('{"schemaVersion":1,"updateCheck":true}', encoding="utf-8")

    incomplete = update_service.evaluate_readiness(directory)
    assert incomplete["ready"] is False
    assert incomplete["serviceMarkerIsNotLiveness"] is True
    missing = [item["name"] for item in incomplete["checks"] if item["status"] == "missing"]
    assert "host_units" in missing
    assert any(item.get("repair") for item in incomplete["checks"] if item["name"] == "host_units")

    _marker(directory, units=list(update_service.HOST_UPDATE_UNITS))
    ready = update_service.evaluate_readiness(directory)
    assert ready["ready"] is True
    assert any(item["status"] == "declared" for item in ready["checks"])
