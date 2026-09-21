"""File-protocol client for the host-side pib update runner.

This module deliberately contains no Flask imports.  The web process only writes
requests; the systemd-triggered host runner performs the destructive work.
"""

from __future__ import annotations

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import tempfile
from typing import Any, Mapping
from uuid import uuid4

DEFAULT_UPDATE_DIR = "/app/.update"
CONFIRMATION_TOKEN = "UPDATE"
ALLOWED_CHANNELS = frozenset({"release", "develop"})

RUNNER_STATES = (
    "preflight",
    "fetching",
    "building",
    "restarting",
    "migrating",
    "verifying",
    "done",
    "failed",
    "rolled_back",
)
ACTIVE_STATES = frozenset(RUNNER_STATES[:6])
TERMINAL_STATES = frozenset(RUNNER_STATES[6:])


class UpdateValidationError(ValueError):
    """Raised when an update request is not safe to enqueue."""


class UpdateNotInstalledError(RuntimeError):
    """Raised when the host update service's shared directory is absent."""


class UpdateConflictError(RuntimeError):
    """Raised when an update is already queued or running."""

    def __init__(self, status: Mapping[str, Any]):
        super().__init__("An update is already queued or running")
        self.status = dict(status)


def update_directory() -> Path:
    return Path(os.environ.get("PIB_UPDATE_DIR", DEFAULT_UPDATE_DIR))


def validate_request_fields(
    channel: Any, force: Any, confirmation: Any, actor: Any
) -> tuple[str, bool, str]:
    """Validate user-controlled request fields without doing any I/O."""
    if channel is None:
        channel = "release"
    if channel not in ALLOWED_CHANNELS:
        raise UpdateValidationError(
            "channel must be one of: " + ", ".join(sorted(ALLOWED_CHANNELS))
        )
    if type(force) is not bool:
        raise UpdateValidationError("force must be a boolean")
    if confirmation != CONFIRMATION_TOKEN:
        raise UpdateValidationError(
            f"confirmation must exactly match {CONFIRMATION_TOKEN!r}"
        )
    if not isinstance(actor, str) or not actor.strip():
        raise UpdateValidationError("actor must be a non-empty string")
    return channel, force, actor.strip()


def build_request(
    *,
    channel: Any = "release",
    force: Any = False,
    confirmation: Any,
    actor: Any,
    job_id: str | None = None,
    requested_at: str | None = None,
) -> dict[str, Any]:
    """Build a validated request document."""
    channel, force, actor = validate_request_fields(channel, force, confirmation, actor)
    job_id = job_id or str(uuid4())
    requested_at = requested_at or datetime.now(timezone.utc).isoformat()
    if not isinstance(job_id, str) or not job_id.strip():
        raise UpdateValidationError("job id must be a non-empty string")
    if not isinstance(requested_at, str) or not requested_at.strip():
        raise UpdateValidationError("timestamp must be a non-empty string")
    return {
        "schemaVersion": 1,
        "jobId": job_id,
        "requestedAt": requested_at,
        "actor": actor,
        "channel": channel,
        "force": force,
        "confirmation": confirmation,
    }


def classify_state(
    status: Mapping[str, Any] | None, request_pending: bool = False
) -> str:
    """Classify runner state for clients without consulting the filesystem."""
    state = status.get("state") if status else None
    if state in ACTIVE_STATES:
        return "running"
    if request_pending:
        return "queued"
    if status is None:
        return "idle"
    if state == "done":
        return "succeeded"
    if state in {"failed", "rolled_back"}:
        return state
    return "unknown"


def is_active(status: Mapping[str, Any] | None, request_pending: bool = False) -> bool:
    if status and status.get("classification") in {"queued", "running"}:
        return True
    return classify_state(status, request_pending) in {"queued", "running"}


def evaluate_update_available(
    installed: Mapping[str, Any], targets: Mapping[str, Any]
) -> dict[str, bool | str]:
    """Compare repository SHAs; unknown inputs produce an unknown answer."""
    result: dict[str, bool | str] = {}
    for repository in sorted(set(installed) | set(targets)):
        current = installed.get(repository)
        target = targets.get(repository)
        if (
            not isinstance(current, str)
            or not isinstance(target, str)
            or current == "unknown"
            or target == "unknown"
        ):
            result[repository] = "unknown"
        else:
            result[repository] = current != target
    return result


def atomic_write_json(path: Path, document: Mapping[str, Any]) -> None:
    """Durably replace a JSON protocol file without exposing partial content."""
    path.parent.mkdir(parents=False, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", dir=path.parent
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as temporary:
            json.dump(document, temporary, sort_keys=True)
            temporary.write("\n")
            temporary.flush()
            os.fsync(temporary.fileno())
        # mkstemp creates 0600 owned by whoever runs this process - the flask
        # container runs as root, while the host runner executes as user pib.
        # Group read/write on the shared directory (which is created setgid, so
        # new files inherit the pib group) is what lets the runner read the
        # request at all.
        os.chmod(temporary_name, 0o660)
        os.replace(temporary_name, path)
    except BaseException:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise


def _read_json(path: Path) -> dict[str, Any] | None:
    try:
        with path.open(encoding="utf-8") as source:
            document = json.load(source)
    except FileNotFoundError:
        return None
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        return {"state": "failed", "error": f"Cannot read {path.name}: {error}"}
    if not isinstance(document, dict):
        return {"state": "failed", "error": f"{path.name} is not a JSON object"}
    return document


def get_status(directory: Path | None = None) -> dict[str, Any]:
    directory = directory or update_directory()
    if not directory.is_dir():
        return {
            "state": "not_installed",
            "classification": "not_installed",
            "error": f"Host update service is not installed at {directory}",
        }
    status = _read_json(directory / "status.json")
    pending = (directory / "request.json").is_file()
    if status is None:
        response: dict[str, Any] = {
            "state": "queued" if pending else "idle",
        }
        queued_request = _read_json(directory / "request.json") if pending else None
        if queued_request and "error" not in queued_request:
            response.update(
                {
                    key: queued_request[key]
                    for key in ("jobId", "channel", "requestedAt")
                    if key in queued_request
                }
            )
    else:
        response = dict(status)
    response["classification"] = classify_state(status, pending)
    response["requestPending"] = pending
    response["cancelRequested"] = (directory / "cancel.json").is_file()
    return response


def enqueue_update(
    request_document: Mapping[str, Any], directory: Path | None = None
) -> dict[str, Any]:
    directory = directory or update_directory()
    if not directory.is_dir():
        raise UpdateNotInstalledError(
            f"Host update service is not installed at {directory}"
        )
    status = get_status(directory)
    if is_active(status, status.get("requestPending", False)):
        raise UpdateConflictError(status)
    (directory / "cancel.json").unlink(missing_ok=True)
    (directory / "status.json").unlink(missing_ok=True)
    atomic_write_json(directory / "request.json", request_document)
    return get_status(directory)


def request_cancel(directory: Path | None = None) -> dict[str, Any]:
    directory = directory or update_directory()
    if not directory.is_dir():
        raise UpdateNotInstalledError(
            f"Host update service is not installed at {directory}"
        )
    status = get_status(directory)
    if not is_active(status, status.get("requestPending", False)):
        raise UpdateConflictError(status)
    atomic_write_json(
        directory / "cancel.json",
        {
            "requestedAt": datetime.now(timezone.utc).isoformat(),
            "jobId": status.get("jobId", "unknown"),
        },
    )
    status["cancelRequested"] = True
    return status


def read_log(offset: int, directory: Path | None = None) -> dict[str, Any]:
    if type(offset) is not int or offset < 0:
        raise UpdateValidationError("offset must be a non-negative integer")
    directory = directory or update_directory()
    if not directory.is_dir():
        raise UpdateNotInstalledError(
            f"Host update service is not installed at {directory}"
        )
    log_path = directory / "update.log"
    try:
        size = log_path.stat().st_size
        actual_offset = min(offset, size)
        with log_path.open("rb") as log_file:
            log_file.seek(actual_offset)
            content = log_file.read()
    except FileNotFoundError:
        size = 0
        actual_offset = 0
        content = b""
    return {
        "offset": actual_offset,
        "nextOffset": actual_offset + len(content),
        "content": content.decode("utf-8", errors="replace"),
        "eof": actual_offset + len(content) >= size,
    }


def program_running_signal() -> bool | None:
    """Return authoritative running state, or None when no signal exists.

    No lock, database field, container state, or API in the current backend
    distinguishes an executing user program from an idle ros-programs process.
    This explicit hook prevents treating container liveness as execution state.
    """
    return None
