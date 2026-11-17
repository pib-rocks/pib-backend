"""The Docker purge preserves tagged images and prunes the BuildKit cache.

Tagged Compose images must survive container purge so the boot recovery service
can recreate the PIB stacks without pulling from a registry. POST /build/prune
targets the BuildKit cache rather than the legacy builder.
"""

from __future__ import annotations

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
        return responses.get(path.split("?")[0], (200, b"{}"))

    return _request, calls, timeouts


def test_purge_keeps_tagged_images_and_prunes_the_buildkit_cache(monkeypatch):
    responses = {
        "/containers/prune": (200, json.dumps({"SpaceReclaimed": 1000}).encode()),
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
    assert any(p.startswith("/build/prune") and "all=true" in p for p in calls)
    assert body["reclaimedBytes"] == 11_001_000
    assert body["status"] == "success"
    assert "freed" in body["message"]


def test_purge_reports_a_failing_step_and_still_runs_the_others(monkeypatch):
    responses = {
        "/images/prune": (500, b'{"message": "boom"}'),
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


def test_purge_survives_an_unreadable_response_body(monkeypatch):
    responses = {"/images/prune": (200, b"not json")}
    request, _calls, _timeouts = _fake_engine(responses)
    monkeypatch.setattr(docker_admin_service, "_docker_request", request)

    _code, body = docker_admin_service.purge_docker()

    assert body["details"]["images"] == "cleared"
    assert body["reclaimedBytes"] == 0
