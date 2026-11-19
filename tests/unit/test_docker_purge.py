"""The Docker purge preserves tagged images and prunes the BuildKit cache.

Images referenced by the saved Compose deployment must survive container purge
so boot recovery can recreate the PIB stacks without pulling from a registry.
Unused tagged images and dangling images can be removed. POST /build/prune targets
the BuildKit cache rather than the legacy builder.
"""

from __future__ import annotations

import fcntl
import json
import sys
from pathlib import Path
from urllib.parse import unquote

REPO_ROOT = Path(__file__).resolve().parents[2]
FLASK_DIR = REPO_ROOT / "pib_api" / "flask"
if str(FLASK_DIR) not in sys.path:
    sys.path.insert(0, str(FLASK_DIR))

from service import docker_admin_service  # noqa: E402


def _fake_engine(responses):
    calls: list[str] = []
    timeouts: list[float] = []

    def _request(method, path, payload=None, timeout=10.0):
        calls.append(path)
        timeouts.append(timeout)
        default = (
            (200, b"[]") if path.split("?")[0] == "/containers/json" else (200, b"{}")
        )
        return responses.get(path.split("?")[0], default)

    return _request, calls, timeouts


def _write_deployment_state(tmp_path: Path, monkeypatch) -> set[str]:
    update_dir = tmp_path / "update"
    update_dir.mkdir(exist_ok=True)
    monkeypatch.setenv("PIB_UPDATE_DIR", str(update_dir))
    references = {
        "registry.example/pib/backend:v2.0.0",
        "registry.example/pib/cerebra:v2.0.0",
    }
    document = {
        "schemaVersion": 1,
        "projects": [
            {
                "name": "pib-backend",
                "composeConfig": {
                    "services": {
                        "flask": {"image": "registry.example/pib/backend:v2.0.0"}
                    }
                },
            },
            {
                "name": "cerebra",
                "composeConfig": {
                    "services": {
                        "app": {"image": "registry.example/pib/cerebra:v2.0.0"}
                    }
                },
            },
        ],
    }
    (update_dir / "deployments.json").write_text(json.dumps(document), encoding="utf-8")
    return references


def test_purge_keeps_current_compose_images_and_prunes_obsolete_tags(
    monkeypatch, tmp_path
):
    references = _write_deployment_state(tmp_path, monkeypatch)
    responses = {
        "/containers/prune": (200, json.dumps({"SpaceReclaimed": 1000}).encode()),
        "/images/json": (
            200,
            json.dumps(
                [
                    {
                        "RepoTags": [
                            "registry.example/pib/backend:v2.0.0",
                            "registry.example/pib/backend:v1.9.0",
                        ]
                    },
                    {
                        "RepoTags": [
                            "registry.example/pib/cerebra:v2.0.0",
                            "registry.example/pib/cerebra:v1.9.0",
                        ]
                    },
                    {"RepoTags": ["old/unused:latest"]},
                    {"RepoTags": ["<none>:<none>"]},
                ]
            ).encode(),
        ),
        "/images/prune": (200, json.dumps({"SpaceReclaimed": 8_000_000}).encode()),
        "/volumes/prune": (200, json.dumps({"SpaceReclaimed": 0}).encode()),
        "/networks/prune": (200, json.dumps({"SpaceReclaimed": 0}).encode()),
        "/build/prune": (200, json.dumps({"SpaceReclaimed": 3_000_000}).encode()),
    }
    request, calls, timeouts = _fake_engine(responses)
    monkeypatch.setattr(docker_admin_service, "_docker_request", request)

    code, body = docker_admin_service.purge_docker()

    assert code == 200
    assert all(t >= 600 for t in timeouts), timeouts
    images_call = next(p for p in calls if p.startswith("/images/prune"))
    assert "filters=" in images_call
    payload = json.loads(unquote(images_call.split("filters=", 1)[1]))
    assert payload == {"dangling": ["true"]}
    deleted = {
        unquote(p.split("?", 1)[0].removeprefix("/images/"))
        for p in calls
        if p.startswith("/images/")
        and p != images_call
        and p.split("?", 1)[0] != "/images/json"
    }
    assert deleted == {
        "registry.example/pib/backend:v1.9.0",
        "registry.example/pib/cerebra:v1.9.0",
        "old/unused:latest",
    }
    assert references.isdisjoint(deleted)
    assert any(p.startswith("/build/prune") and "all=true" in p for p in calls)
    assert body["reclaimedBytes"] == 11_001_000
    assert body["status"] == "success"
    assert "freed" in body["message"]


def test_purge_preserves_running_container_image_by_id_even_without_current_tag(
    monkeypatch, tmp_path
):
    _write_deployment_state(tmp_path, monkeypatch)
    responses = {
        "/containers/json": (
            200,
            json.dumps([{"ImageID": "sha256:active-image"}]).encode(),
        ),
        "/images/json": (
            200,
            json.dumps(
                [
                    {
                        "Id": "sha256:active-image",
                        "RepoTags": ["registry.example/pib/backend:old-alias"],
                    }
                ]
            ).encode(),
        ),
        "/images/prune": (200, b"{}"),
    }
    request, calls, _timeouts = _fake_engine(responses)
    monkeypatch.setattr(docker_admin_service, "_docker_request", request)

    code, body = docker_admin_service.purge_docker()

    assert code == 200
    assert body["status"] == "success"
    assert not any(path.startswith("/images/registry.example") for path in calls)


def test_purge_skips_image_cleanup_when_running_container_list_fails(
    monkeypatch, tmp_path
):
    _write_deployment_state(tmp_path, monkeypatch)
    request, calls, _timeouts = _fake_engine({"/containers/json": (500, b"")})
    monkeypatch.setattr(docker_admin_service, "_docker_request", request)

    code, body = docker_admin_service.purge_docker()

    assert code == 200
    assert body["status"] == "partial"
    assert any("running containers" in error for error in body["errors"])
    assert not any(path.startswith("/images/") for path in calls)


def test_purge_matches_implicit_latest_and_digest_references(monkeypatch, tmp_path):
    _write_deployment_state(tmp_path, monkeypatch)
    update_dir = tmp_path / "update"
    state_path = update_dir / "deployments.json"
    document = json.loads(state_path.read_text(encoding="utf-8"))
    document["projects"][0]["composeConfig"]["services"]["flask"][
        "image"
    ] = "registry.example/pib/backend"
    document["projects"][1]["composeConfig"]["services"]["app"][
        "image"
    ] = "registry.example/pib/cerebra@sha256:deadbeef"
    state_path.write_text(json.dumps(document), encoding="utf-8")
    responses = {
        "/images/json": (
            200,
            json.dumps(
                [
                    {
                        "RepoTags": [
                            "registry.example/pib/backend:latest",
                            "registry.example/pib/backend:v1.9.0",
                        ],
                        "RepoDigests": [],
                    },
                    {
                        "RepoTags": [],
                        "RepoDigests": ["registry.example/pib/cerebra@sha256:deadbeef"],
                    },
                    {
                        "RepoTags": ["old/unused:latest"],
                        "RepoDigests": [],
                    },
                ]
            ).encode(),
        ),
        "/build/prune": (200, b"{}"),
    }
    request, calls, _timeouts = _fake_engine(responses)
    monkeypatch.setattr(docker_admin_service, "_docker_request", request)

    code, body = docker_admin_service.purge_docker()

    deleted = {
        unquote(path.split("?", 1)[0].removeprefix("/images/"))
        for path in calls
        if path.startswith("/images/")
        and path.split("?", 1)[0] not in {"/images/json"}
        and not path.startswith("/images/prune")
    }
    assert code == 200
    assert deleted == {
        "registry.example/pib/backend:v1.9.0",
        "old/unused:latest",
    }
    assert not any(path.startswith("/images/prune") for path in calls)
    assert body["status"] == "success"


def test_purge_reports_a_failing_step_and_still_runs_the_others(monkeypatch, tmp_path):
    _write_deployment_state(tmp_path, monkeypatch)
    responses = {
        "/images/prune": (500, b'{"message": "boom"}'),
        "/images/json": (200, b"[]"),
        "/build/prune": (200, json.dumps({"SpaceReclaimed": 2_000}).encode()),
    }
    request, calls, _timeouts = _fake_engine(responses)
    monkeypatch.setattr(docker_admin_service, "_docker_request", request)

    code, body = docker_admin_service.purge_docker()

    assert code == 200
    assert body["status"] == "partial"
    assert any("images" in error for error in body["errors"])
    assert any(p.startswith("/build/prune") for p in calls)
    assert body["reclaimedBytes"] == 2_000
    assert "failed" in body["message"]


def test_purge_survives_an_unreadable_response_body(monkeypatch, tmp_path):
    _write_deployment_state(tmp_path, monkeypatch)
    responses = {"/images/prune": (200, b"not json"), "/images/json": (200, b"[]")}
    request, _calls, _timeouts = _fake_engine(responses)
    monkeypatch.setattr(docker_admin_service, "_docker_request", request)

    _code, body = docker_admin_service.purge_docker()

    assert body["details"]["images"]["SpaceReclaimed"] == 0
    assert body["reclaimedBytes"] == 0


def test_purge_keeps_all_tags_when_recovery_state_is_missing(monkeypatch, tmp_path):
    update_dir = tmp_path / "update"
    update_dir.mkdir()
    monkeypatch.setenv("PIB_UPDATE_DIR", str(update_dir))
    responses = {
        "/containers/prune": (200, b"{}"),
        "/volumes/prune": (200, b"{}"),
        "/networks/prune": (200, b"{}"),
        "/build/prune": (200, b"{}"),
    }
    request, calls, _timeouts = _fake_engine(responses)
    monkeypatch.setattr(docker_admin_service, "_docker_request", request)

    code, body = docker_admin_service.purge_docker()

    assert code == 200
    assert body["status"] == "partial"
    assert "images" in body["errors"][0]
    assert not any(path.startswith("/images/") for path in calls)


def test_purge_keeps_all_tags_when_recovery_state_is_incomplete(monkeypatch, tmp_path):
    _write_deployment_state(tmp_path, monkeypatch)
    update_dir = tmp_path / "update"
    state_path = update_dir / "deployments.json"
    document = json.loads(state_path.read_text(encoding="utf-8"))
    document["projects"] = document["projects"][:1]
    state_path.write_text(json.dumps(document), encoding="utf-8")
    request, calls, _timeouts = _fake_engine({})
    monkeypatch.setattr(docker_admin_service, "_docker_request", request)

    code, body = docker_admin_service.purge_docker()

    assert code == 200
    assert body["status"] == "partial"
    assert "both PIB Compose projects" in body["errors"][0]
    assert not any(path.startswith("/images/") for path in calls)


def test_purge_refuses_to_run_while_deployment_lock_is_held(monkeypatch, tmp_path):
    update_dir = tmp_path / "update"
    update_dir.mkdir()
    monkeypatch.setenv("PIB_UPDATE_DIR", str(update_dir))
    lock_path = update_dir / "deployment.lock"
    with lock_path.open("a", encoding="utf-8") as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
        try:
            request, calls, _timeouts = _fake_engine({})
            monkeypatch.setattr(docker_admin_service, "_docker_request", request)

            code, body = docker_admin_service.purge_docker()
        finally:
            fcntl.flock(lock.fileno(), fcntl.LOCK_UN)

    assert code == 409
    assert body["status"] == "error"
    assert calls == []
