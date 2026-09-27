"""Request-file contract for the display web surface."""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
DISPLAY_ROOT = REPO_ROOT / "ros_packages" / "display"
sys.path.insert(0, str(DISPLAY_ROOT))

import display.display_web_request as display_web_request  # noqa: E402
from display.display_web_request import (  # noqa: E402
    REQUEST_FILENAME,
    STATUS_FILENAME,
    SURFACE_READY,
    SURFACE_WEB,
    SURFACE_WEB_FAILED,
    acknowledge_web_status,
    command_effect,
    read_status,
    validate_document,
    write_hide_request,
    write_open_request,
)

MODULE_PATH = Path(display_web_request.__file__).resolve()


def test_module_resolves_to_this_worktree():
    expected = (
        REPO_ROOT / "ros_packages" / "display" / "display" / "display_web_request.py"
    ).resolve()
    assert MODULE_PATH == expected


def test_open_request_matches_the_host_schema(tmp_path: Path):
    document = write_open_request(tmp_path, "http://localhost")
    stored = json.loads((tmp_path / REQUEST_FILENAME).read_text(encoding="utf-8"))
    assert stored == document
    assert validate_document(stored) == stored
    assert stored["schemaVersion"] == 1
    assert stored["action"] == "open"
    assert stored["url"] == "http://localhost"
    assert stored["requestedAt"].endswith("Z")


def test_hide_request_is_distinguishable_from_open(tmp_path: Path):
    opened = write_open_request(tmp_path, "http://localhost/cerebra")
    hidden = write_hide_request(tmp_path)
    assert opened["action"] == "open"
    assert opened["url"] == "http://localhost/cerebra"
    assert hidden["action"] == "hide"
    assert hidden["url"] == ""
    assert hidden["action"] != opened["action"]
    stored = json.loads((tmp_path / REQUEST_FILENAME).read_text(encoding="utf-8"))
    assert stored["action"] == "hide"


def test_write_is_atomic_and_leaves_no_temporary_file(tmp_path: Path, monkeypatch):
    seen: list[Path] = []
    real_replace = os.replace

    def wrapping_replace(source, destination):
        source_path = Path(source)
        payload = json.loads(source_path.read_text(encoding="utf-8"))
        assert payload["schemaVersion"] == 1
        assert source_path.name.startswith(".display-web.json.")
        seen.append(source_path)
        return real_replace(source, destination)

    monkeypatch.setattr(os, "replace", wrapping_replace)
    write_open_request(tmp_path, "https://example.com/page")
    assert seen
    assert list(tmp_path.glob(".display-web.json.*")) == []
    assert (tmp_path / REQUEST_FILENAME).is_file()


def test_writer_does_not_remove_the_request_or_change_permissions(
    tmp_path: Path, monkeypatch
):
    removed: list[str] = []
    real_unlink = os.unlink

    def tracking_unlink(path, *args, **kwargs):
        removed.append(Path(path).name)
        return real_unlink(path, *args, **kwargs)

    def forbidden(*_args, **_kwargs):
        raise AssertionError("permission change")

    monkeypatch.setattr(os, "unlink", tracking_unlink)
    monkeypatch.setattr(os, "chmod", forbidden)
    monkeypatch.setattr(os, "chown", forbidden, raising=False)
    (tmp_path / "request.json").write_text('{"jobId":"leave-me"}\n', encoding="utf-8")
    write_open_request(tmp_path, "http://localhost")
    assert (tmp_path / REQUEST_FILENAME).is_file()
    assert REQUEST_FILENAME not in removed
    assert (tmp_path / "request.json").read_text(
        encoding="utf-8"
    ) == '{"jobId":"leave-me"}\n'


def test_invalid_url_is_rejected_without_a_request_file(tmp_path: Path):
    with pytest.raises(ValueError, match="url"):
        write_open_request(tmp_path, "not a url")
    assert not (tmp_path / REQUEST_FILENAME).exists()


def test_existing_draw_commands_are_unchanged_until_the_web_surface_is_open():
    for kind in ("text", "expression", "raw"):
        assert command_effect(False, kind) == "draw"
        assert command_effect(True, kind) == "suppress"
    assert command_effect(False, "hide") == "hide"
    assert command_effect(True, "hide") == "hide"


def test_host_status_drives_the_ready_channel():
    pending_at = "2026-09-28T00:00:00Z"
    opened = acknowledge_web_status(
        "open",
        pending_at,
        {"state": "done", "action": "open", "requestedAt": pending_at},
        timed_out=False,
        failure_reported=False,
    )
    assert opened.surface == SURFACE_WEB
    assert opened.suppression == "hold"
    assert opened.clear_pending is True

    hidden = acknowledge_web_status(
        "hide",
        pending_at,
        {"state": "done", "action": "hide", "requestedAt": pending_at},
        timed_out=False,
        failure_reported=False,
    )
    assert hidden.surface == SURFACE_READY
    assert hidden.suppression == "release"

    failed = acknowledge_web_status(
        "open",
        pending_at,
        {"state": "failed", "action": "open", "requestedAt": pending_at},
        timed_out=False,
        failure_reported=False,
    )
    assert failed.surface == SURFACE_WEB_FAILED
    assert failed.suppression == "release"
    assert failed.clear_pending is True

    waiting = acknowledge_web_status(
        "open", pending_at, None, timed_out=False, failure_reported=False
    )
    assert waiting.surface is None
    assert waiting.clear_pending is False

    timed_out = acknowledge_web_status(
        "open", pending_at, None, timed_out=True, failure_reported=False
    )
    assert timed_out.surface == SURFACE_WEB_FAILED
    assert timed_out.clear_pending is False

    already = acknowledge_web_status(
        "open", pending_at, None, timed_out=True, failure_reported=True
    )
    assert already.surface is None


def test_status_reader_ignores_a_missing_file(tmp_path: Path):
    assert read_status(tmp_path) is None


def test_units_are_a_separate_pair_and_the_compose_file_mounts_the_directory():
    service = (REPO_ROOT / "setup/setup_files/pib-display-web.service").read_text(
        encoding="utf-8"
    )
    path_unit = (REPO_ROOT / "setup/setup_files/pib-display-web.path").read_text(
        encoding="utf-8"
    )
    runner = (REPO_ROOT / "setup/display_web_runner.sh").read_text(encoding="utf-8")
    installer = (REPO_ROOT / "setup/installation_scripts/docker_install.sh").read_text(
        encoding="utf-8"
    )
    setup = (REPO_ROOT / "setup/setup-pib.sh").read_text(encoding="utf-8")
    compose = (REPO_ROOT / "docker-compose.yaml").read_text(encoding="utf-8")
    display = compose.split("  ros-display:", 1)[1].split("\n  ros-audio-io:", 1)[0]

    assert "Unit=pib-display-web.service" in path_unit
    assert "display-web.json" in path_unit
    assert "request.json" not in path_unit
    assert "pib-update.service" not in service
    assert "pib-update.service" not in path_unit
    assert "pib-update.service" not in runner
    assert "Type=oneshot" in service
    assert "User=pib" in service
    assert "Group=pib" in service
    assert "KillMode=none" in service
    assert "display_web_runner.sh" in service
    assert "chmod" not in runner
    assert "chown" not in runner
    assert STATUS_FILENAME in runner
    assert "function setup_display_web_service" in installer
    assert "setup_display_web_service ||" in installer
    assert "docker_install.sh" in setup
    assert "/home/pib/app/.update:/app/.update" in display
    assert "PIB_UPDATE_DIR=/app/.update" in display
