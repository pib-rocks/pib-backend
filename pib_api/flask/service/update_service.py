"""File-protocol client for the host-side pib update runner.

This module deliberately contains no Flask imports.  The web process only writes
requests; the systemd-triggered host runner performs the destructive work.
"""

from __future__ import annotations

from datetime import datetime, timezone
import importlib.util
import json
import os
from pathlib import Path
import re
import tempfile
from typing import Any, Mapping
from uuid import uuid4

DEFAULT_UPDATE_DIR = "/app/.update"
SERVICE_MARKER_NAME = "service.json"
EXECUTOR_LIVENESS_NAME = "executor.json"
INTERRUPTED_JOB_NAME = "interrupted.json"
UPDATE_CHECK_MARKER_FIELD = "updateCheck"
CONFIRMATION_TOKEN = "UPDATE"
ALLOWED_CHANNELS = frozenset({"release", "develop"})
UPDATE_REPOSITORIES = ("pib-backend", "cerebra")
HOST_UPDATE_UNITS = (
    "pib-update.path",
    "pib-update.service",
    "pib-update-check.path",
    "pib-update-check.service",
)
CANCEL_SAFE_STATES = frozenset({"queued", "preflight", "fetching"})
CHECK_ID_PATTERN = re.compile(r"^[0-9a-fA-F-]{36}$")
STABLE_TAG_PATTERN = re.compile(r"^v(0|[1-9]\d*)\.(0|[1-9]\d*)\.(0|[1-9]\d*)$")
COMMIT_PATTERN = re.compile(r"^[0-9a-f]{40}$")
_RELEASES_MODULE = None
# Queued work with no executor start, and a heartbeat that has stopped.
# In-progress jobs from a runner that does not write executor.json stay active:
# a long source build must not be classified as dead merely because status.json
# is quiet between phases.
DEFAULT_START_DEADLINE_SECONDS = 180
DEFAULT_HEARTBEAT_DEADLINE_SECONDS = 120

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
    """Raised when the host update service is absent or not fully installed.

    ``state`` is ``not_installed`` when the shared directory is missing and
    ``runner_missing`` when the directory exists (docker creates the bind-mount
    point on its own) but the installer never placed its marker.
    """

    def __init__(self, message: str, state: str = "not_installed") -> None:
        super().__init__(message)
        self.state = state


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


def build_check_request(
    *,
    channel: Any = "release",
    actor: Any,
    requested_at: str | None = None,
    check_id: str | None = None,
) -> dict[str, Any]:
    """Build a validated side-effect-free availability check request.

    ``checkId`` identifies this check. A later ``available.json`` belongs to
    the check only when it carries the same id. Callers that omit the channel
    still default to ``release`` at the controller; this function does not
    invent a channel from a device that asked for ``develop``.
    """
    if channel not in ALLOWED_CHANNELS:
        raise UpdateValidationError(
            "channel must be one of: " + ", ".join(sorted(ALLOWED_CHANNELS))
        )
    if not isinstance(actor, str) or not actor.strip():
        raise UpdateValidationError("actor must be a non-empty string")
    requested_at = requested_at or datetime.now(timezone.utc).isoformat()
    if not isinstance(requested_at, str) or not requested_at.strip():
        raise UpdateValidationError("timestamp must be a non-empty string")
    check_id = check_id or str(uuid4())
    if not isinstance(check_id, str) or not CHECK_ID_PATTERN.fullmatch(check_id):
        raise UpdateValidationError("check id must be a UUID")
    return {
        "schemaVersion": 1,
        "checkId": check_id,
        "actor": actor.strip(),
        "channel": channel,
        "requestedAt": requested_at,
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
    """True only for a job that still owns the executor.

    A ``stale`` classification is the PR-1812 outcome: the document is still
    visible, but it must not block a later update. There is no second lock.
    """
    if status and status.get("classification") == "stale":
        return False
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


def has_service_marker(directory: Path | None = None) -> bool:
    """True when the installer has placed its marker in the shared directory."""
    directory = directory or update_directory()
    return (directory / SERVICE_MARKER_NAME).is_file()


def _parse_timestamp(value: object) -> datetime | None:
    if not isinstance(value, str) or not value.strip():
        return None
    text = value.strip()
    if text.endswith("Z"):
        text = text[:-1] + "+00:00"
    try:
        parsed = datetime.fromisoformat(text)
    except ValueError:
        return None
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=timezone.utc)
    return parsed.astimezone(timezone.utc)


def _deadline_seconds(name: str, default: int) -> int:
    raw = os.environ.get(name, str(default))
    try:
        value = int(raw)
    except (TypeError, ValueError):
        return default
    if value < 1:
        return default
    return value


def _check_result(
    name: str, status: str, detail: str, repair: str | None = None
) -> dict[str, str]:
    item = {"name": name, "status": status, "detail": detail}
    if repair:
        item["repair"] = repair
    return item


def evaluate_readiness(directory: Path | None = None) -> dict[str, Any]:
    """Report installer prerequisites. ``service.json`` alone is not liveness.

    Unit activation is attested by the installer marker. The API process does
    not call systemctl. A request that is accepted but never started becomes
    ``stale`` via the start deadline instead.
    """
    directory = directory or update_directory()
    checks: list[dict[str, str]] = []
    if not directory.is_dir():
        checks.append(
            _check_result(
                "shared_directory",
                "missing",
                f"Host update directory is not present at {directory}",
                "Run the update section of setup/installation_scripts/docker_install.sh "
                "so /home/pib/app/.update exists, is group-writable, and is mounted "
                "into the API container. The API cannot create the host executor.",
            )
        )
        return {"ready": False, "checks": checks, "serviceMarkerIsNotLiveness": True}

    writable = os.access(directory, os.W_OK | os.X_OK)
    checks.append(
        _check_result(
            "shared_directory",
            "ok" if writable else "failed",
            (
                "Shared update directory is mounted and writable."
                if writable
                else f"{directory} is not writable by the API process."
            ),
            (
                None
                if writable
                else "On the host: sudo install -d -o pib -g pib -m 2770 /home/pib/app/.update"
            ),
        )
    )
    marker_path = directory / SERVICE_MARKER_NAME
    marker = _read_json(marker_path) if marker_path.is_file() else None
    if not isinstance(marker, dict) or "error" in marker and "runner" not in marker:
        checks.append(
            _check_result(
                "service_marker",
                "missing",
                f"Missing installer marker {SERVICE_MARKER_NAME}.",
                "Re-run the update section of setup/installation_scripts/docker_install.sh. "
                "Docker creates the bind-mount directory on its own; that is not an installed runner.",
            )
        )
        return {
            "ready": False,
            "checks": checks,
            "serviceMarkerIsNotLiveness": True,
        }

    runner = marker.get("runner")
    if isinstance(runner, str) and runner.startswith("/"):
        if os.path.isfile(runner):
            executable = os.access(runner, os.X_OK)
            checks.append(
                _check_result(
                    "runner",
                    "ok" if executable else "failed",
                    (
                        f"Runner is executable at {runner}."
                        if executable
                        else f"Runner exists but is not executable: {runner}."
                    ),
                    (
                        None
                        if executable
                        else "Keep setup/update_runner.sh executable in git (mode 100755). "
                        "Do not chmod a live checkout; a mode change is a dirty file."
                    ),
                )
            )
        else:
            checks.append(
                _check_result(
                    "runner",
                    "declared",
                    f"Installer recorded host runner {runner}. That path is outside "
                    "the API mount, so this process does not claim it is executable.",
                )
            )
    else:
        checks.append(
            _check_result(
                "runner",
                "missing",
                "service.json does not record an absolute host runner path.",
                "Re-run setup/installation_scripts/docker_install.sh so the marker records the runner.",
            )
        )

    units = marker.get("units")
    if isinstance(units, list) and all(unit in units for unit in HOST_UPDATE_UNITS):
        checks.append(
            _check_result(
                "host_units",
                "attested",
                "Installer marker records pib-update.path, pib-update.service, "
                "pib-update-check.path and pib-update-check.service. "
                "Live activation is judged by whether a queued job starts, not by this file.",
            )
        )
    else:
        checks.append(
            _check_result(
                "host_units",
                "missing",
                "service.json does not record the host update units.",
                "Re-run the update section of setup/installation_scripts/docker_install.sh "
                "on the host. Queuing an update cannot install missing systemd units.",
            )
        )

    if marker.get(UPDATE_CHECK_MARKER_FIELD) is True:
        checks.append(
            _check_result(
                "update_check",
                "ok",
                "Availability check runner is advertised by the installer marker.",
            )
        )
    else:
        checks.append(
            _check_result(
                "update_check",
                "missing",
                "Availability check runner is not advertised.",
                "Re-run setup/installation_scripts/docker_install.sh to install pib-update-check.path.",
            )
        )
    ready = all(item["status"] in {"ok", "attested", "declared"} for item in checks)
    return {"ready": ready, "checks": checks, "serviceMarkerIsNotLiveness": True}


def _matching_heartbeat(directory: Path, job_id: object) -> datetime | None:
    document = _read_json(directory / EXECUTOR_LIVENESS_NAME)
    if not isinstance(document, dict) or "error" in document:
        return None
    if job_id is None or document.get("jobId") != job_id:
        return None
    return _parse_timestamp(document.get("updatedAt"))


def _mark_stale(response: dict[str, Any], reason: str, recovery: str) -> None:
    response["classification"] = "stale"
    response["staleReason"] = reason
    response["recovery"] = recovery
    response["blocksNewUpdate"] = False


def _apply_liveness(
    response: dict[str, Any],
    directory: Path,
    pending: bool,
    now: datetime,
) -> None:
    """Overlay PR-1812 staleness onto an otherwise ordinary status document."""
    state = response.get("state")
    job_id = response.get("jobId")
    heartbeat_at = _matching_heartbeat(directory, job_id)
    heartbeat_deadline = _deadline_seconds(
        "PIB_UPDATE_HEARTBEAT_DEADLINE_SECONDS", DEFAULT_HEARTBEAT_DEADLINE_SECONDS
    )
    start_deadline = _deadline_seconds(
        "PIB_UPDATE_START_DEADLINE_SECONDS", DEFAULT_START_DEADLINE_SECONDS
    )
    if not pending and state in ACTIVE_STATES:
        _mark_stale(
            response,
            "Nonterminal status has no request.json, so the executor is not running. "
            "This stale job stays visible and does not block a new update.",
            "Start a new update after reading the interrupted result. "
            "No automatic rollback was performed.",
        )
    elif (
        pending
        and heartbeat_at is not None
        and (now - heartbeat_at).total_seconds() > heartbeat_deadline
    ):
        _mark_stale(
            response,
            "The executor heartbeat for this job is older than the liveness deadline.",
            "The runner stopped updating executor.json. Inspect the host update log, "
            "then start a new update. This stale job does not block that request.",
        )
    elif pending and state in {"queued", "idle"} and heartbeat_at is None:
        requested_at = _parse_timestamp(response.get("requestedAt"))
        if (
            requested_at is not None
            and (now - requested_at).total_seconds() > start_deadline
        ):
            _mark_stale(
                response,
                "The accepted request was not started before the executor start deadline.",
                "The host path unit did not start setup/update_runner.sh. "
                "Re-run setup/installation_scripts/docker_install.sh, then start a new update. "
                "This request no longer blocks a new one.",
            )
    response["blocksNewUpdate"] = response.get("classification") in {
        "queued",
        "running",
    }
    response["cancelSafe"] = bool(
        response.get("classification") in {"queued", "running"}
        and response.get("state") in CANCEL_SAFE_STATES
    )
    interrupted = _read_json(directory / INTERRUPTED_JOB_NAME)
    if (
        isinstance(interrupted, dict)
        and interrupted.get("jobId")
        and interrupted.get("jobId") != response.get("jobId")
        and "error" not in interrupted
    ):
        response["interruptedJob"] = interrupted


def has_update_check_runner(directory: Path | None = None) -> bool:
    """True when the installer marker advertises the separate check runner."""
    directory = directory or update_directory()
    marker = _read_json(directory / SERVICE_MARKER_NAME)
    return bool(marker and marker.get(UPDATE_CHECK_MARKER_FIELD) is True)


def get_status(
    directory: Path | None = None, *, now: datetime | None = None
) -> dict[str, Any]:
    directory = directory or update_directory()
    moment = now or datetime.now(timezone.utc)
    if not directory.is_dir():
        response = {
            "state": "not_installed",
            "classification": "not_installed",
            "error": f"Host update service is not installed at {directory}",
            "blocksNewUpdate": False,
            "cancelSafe": False,
        }
        response["readiness"] = evaluate_readiness(directory)
        return response
    if not has_service_marker(directory):
        response = {
            "state": "runner_missing",
            "classification": "runner_missing",
            "error": (
                f"{directory} exists but no host runner is installed there "
                f"(missing {SERVICE_MARKER_NAME}); run the update setup step "
                "(setup/installation_scripts/docker_install.sh) so that the "
                "systemd units and the runner are in place"
            ),
            "blocksNewUpdate": False,
            "cancelSafe": False,
        }
        response["readiness"] = evaluate_readiness(directory)
        return response
    status = _read_json(directory / "status.json")
    if (
        status
        and "error" in status
        and "state" in status
        and status.get("jobId") is None
    ):
        # Unreadable status.json is a failed document, not an active runner state.
        pass
    pending = (directory / "request.json").is_file()
    if status is None or (
        isinstance(status, dict)
        and status.get("state") == "failed"
        and "jobId" not in status
        and "error" in status
    ):
        response = {
            "state": "queued" if pending else "idle",
        }
        if status and "error" in status and not pending:
            response = {"state": "failed", "error": status.get("error")}
        queued_request = _read_json(directory / "request.json") if pending else None
        if queued_request and "error" not in queued_request:
            response.update(
                {
                    key: queued_request[key]
                    for key in (
                        "jobId",
                        "channel",
                        "requestedAt",
                        "release",
                        "targetKind",
                    )
                    if key in queued_request
                }
            )
        classified_status = (
            None if response.get("state") in {"queued", "idle"} else response
        )
    else:
        response = dict(status)
        classified_status = status
    response["classification"] = classify_state(classified_status, pending)
    response["requestPending"] = pending
    response["cancelRequested"] = (directory / "cancel.json").is_file()
    response["readiness"] = evaluate_readiness(directory)
    _apply_liveness(response, directory, pending, moment)
    return response


def enqueue_update(
    request_document: Mapping[str, Any], directory: Path | None = None
) -> dict[str, Any]:
    directory = directory or update_directory()
    status = get_status(directory)
    if status["state"] in {"not_installed", "runner_missing"}:
        raise UpdateNotInstalledError(status["error"], state=status["state"])
    if is_active(status, status.get("requestPending", False)):
        raise UpdateConflictError(status)
    if status.get("classification") == "stale":
        atomic_write_json(
            directory / INTERRUPTED_JOB_NAME,
            {
                "schemaVersion": 1,
                "jobId": status.get("jobId", "unknown"),
                "state": status.get("state"),
                "classification": "stale",
                "staleReason": status.get("staleReason"),
                "channel": status.get("channel"),
            },
        )
    (directory / "cancel.json").unlink(missing_ok=True)
    (directory / "status.json").unlink(missing_ok=True)
    (directory / EXECUTOR_LIVENESS_NAME).unlink(missing_ok=True)
    atomic_write_json(directory / "request.json", request_document)
    return get_status(directory)


def _require_update_check_runner(directory: Path) -> None:
    status = get_status(directory)
    if status["state"] in {"not_installed", "runner_missing"}:
        raise UpdateNotInstalledError(status["error"], state=status["state"])
    if not has_update_check_runner(directory):
        raise UpdateNotInstalledError(
            "The host update availability runner is not installed; run the update "
            "setup step (setup/installation_scripts/docker_install.sh)",
            state="runner_missing",
        )


def enqueue_check(
    request_document: Mapping[str, Any], directory: Path | None = None
) -> dict[str, Any]:
    """Atomically queue a side-effect-free update availability check."""
    directory = directory or update_directory()
    _require_update_check_runner(directory)
    status = get_status(directory)
    if is_active(status, status.get("requestPending", False)):
        raise UpdateConflictError(status)
    atomic_write_json(directory / "check.json", request_document)
    return dict(request_document)


def _unknown_availability() -> dict[str, Any]:
    return {
        "schemaVersion": 1,
        "checkedAt": None,
        "repositories": {
            repository: {
                "installed": "unknown",
                "target": "unknown",
                "updateAvailable": "unknown",
            }
            for repository in UPDATE_REPOSITORIES
        },
    }


def _previous_result(document: dict[str, Any] | None) -> dict[str, Any] | None:
    if not isinstance(document, dict):
        return None
    if (
        "error" in document
        and "repositories" not in document
        and "checkId" not in document
    ):
        return None
    previous = {key: value for key, value in document.items() if key != "previous"}
    return previous


def get_available(directory: Path | None = None) -> dict[str, Any]:
    """Read the check result that belongs to the current check, if one is queued.

    While ``check.json`` exists the previous ``available.json`` is returned
    under ``previous``. It is not the result of the check that is still running.
    """
    directory = directory or update_directory()
    _require_update_check_runner(directory)
    document = _read_json(directory / "available.json")
    check = _read_json(directory / "check.json")
    check_is_request = (
        isinstance(check, dict)
        and check.get("channel") in ALLOWED_CHANNELS
        and "error" not in check
    )
    if check_is_request:
        return {
            "schemaVersion": 1,
            "state": "pending",
            "checkId": check.get("checkId"),
            "channel": check.get("channel"),
            "requestedAt": check.get("requestedAt"),
            "actor": check.get("actor"),
            "previous": _previous_result(document),
        }
    if document is None:
        return _unknown_availability()
    if document.get("state") == "failed" and "repositories" not in document:
        response = _unknown_availability()
        response["error"] = document.get("error", "Cannot read available.json")
        return response
    return document


def request_cancel(directory: Path | None = None) -> dict[str, Any]:
    directory = directory or update_directory()
    status = get_status(directory)
    if status["state"] in {"not_installed", "runner_missing"}:
        raise UpdateNotInstalledError(status["error"], state=status["state"])
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


def _release_module():
    """Load the host pairing helper. It has no Flask dependency."""
    global _RELEASES_MODULE
    if _RELEASES_MODULE is None:
        path = Path(__file__).resolve().parents[3] / "setup" / "update_releases.py"
        spec = importlib.util.spec_from_file_location("update_releases", path)
        if spec is None or spec.loader is None:
            raise UpdateValidationError("release pairing helper is not available")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        _RELEASES_MODULE = module
    return _RELEASES_MODULE


def _commit_map(targets: object) -> dict[str, str]:
    if not isinstance(targets, dict):
        raise UpdateValidationError("confirmed release is missing commit targets")
    resolved: dict[str, str] = {}
    for name in UPDATE_REPOSITORIES:
        entry = targets.get(name)
        commit = entry.get("commit") if isinstance(entry, dict) else entry
        if not isinstance(commit, str) or not COMMIT_PATTERN.fullmatch(commit):
            raise UpdateValidationError(f"confirmed release is missing a {name} commit")
        resolved[name] = commit
    return resolved


def pin_accepted_job(
    request: Mapping[str, Any],
    *,
    release: Any,
    check_id: Any,
    pin: Any,
    directory: Path | None = None,
) -> dict[str, Any]:
    """Copy server-resolved commits into the immutable job.

    Client ``targets`` are never read. A channel-only request must not call
    this: that shape remains the legacy moving-branch update.
    """
    if type(pin) is not bool:
        raise UpdateValidationError("pin must be a boolean")
    directory = directory or update_directory()
    available = _read_json(directory / "available.json")
    if (
        not isinstance(available, dict)
        or "repositories" not in available
        and "releases" not in available
    ):
        raise UpdateValidationError(
            "A completed availability check is required before a pinned install"
        )
    if available.get("state") not in {None, "completed"}:
        raise UpdateValidationError("The availability check has not completed")
    if check_id is not None:
        if not isinstance(check_id, str) or not CHECK_ID_PATTERN.fullmatch(check_id):
            raise UpdateValidationError("checkId must be a UUID")
        if available.get("checkId") != check_id:
            raise UpdateValidationError(
                "The availability check does not match the confirmed check id"
            )
    if (
        isinstance(available.get("channel"), str)
        and available.get("channel") != request["channel"]
    ):
        raise UpdateValidationError(
            "The availability check channel does not match the install request"
        )
    pinned = dict(request)
    if request["channel"] == "release":
        if not isinstance(release, str) or not STABLE_TAG_PATTERN.fullmatch(release):
            raise UpdateValidationError(
                "release must be a stable published tag such as v1.2.3"
            )
        match = None
        for item in available.get("releases") or []:
            if (
                isinstance(item, dict)
                and item.get("tag") == release
                and item.get("installable") is True
            ):
                match = item
                break
        if match is None:
            raise UpdateValidationError(
                "That release is not an installable paired release in the confirmed check"
            )
        pinned["release"] = release
        pinned["targets"] = _commit_map(match.get("targets"))
        pinned["targetKind"] = "published-release"
    else:
        if pin is not True:
            raise UpdateValidationError(
                "A develop install must set pin true so the checked commits are recorded"
            )
        repositories = available.get("repositories")
        if not isinstance(repositories, dict):
            raise UpdateValidationError(
                "The develop check did not pin both repository commits"
            )
        targets = {}
        for name in UPDATE_REPOSITORIES:
            entry = repositories.get(name)
            sha = entry.get("target") if isinstance(entry, dict) else None
            if not isinstance(sha, str) or not COMMIT_PATTERN.fullmatch(sha):
                raise UpdateValidationError(
                    "The develop check did not pin both repository commits"
                )
            targets[name] = sha
        pinned["targets"] = targets
        pinned["targetKind"] = "develop-pin"
    if isinstance(available.get("checkId"), str):
        pinned["checkId"] = available["checkId"]
    return pinned


def annotate_installed_release(
    document: Mapping[str, Any], installed: Mapping[str, Any]
) -> dict[str, Any]:
    """Attach release relations when the check discovered paired tags."""
    if not isinstance(document, dict):
        return dict(document) if isinstance(document, Mapping) else {}
    if document.get("state") == "pending":
        pending = dict(document)
        previous = pending.get("previous")
        if isinstance(previous, dict) and "releases" in previous:
            pending["previous"] = _release_module().annotate_relations(
                previous, installed
            )
        return pending
    if "releases" not in document:
        return dict(document)
    return _release_module().annotate_relations(document, installed)


def program_running_signal() -> bool | None:
    """Return authoritative running state, or None when no signal exists.

    No lock, database field, container state, or API in the current backend
    distinguishes an executing user program from an idle ros-programs process.
    This explicit hook prevents treating container liveness as execution state.
    """
    return None
