"""Side-effect-free update availability comparison for the host runner.

This module is dependency free because it runs on the host, outside Flask.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import re
import shlex
import sys
import tempfile
from typing import Any, Mapping

ALLOWED_CHANNELS = frozenset({"release", "develop"})
CHANNEL_BRANCHES = {"release": "main", "develop": "develop"}
UNKNOWN = "unknown"
SHA_PATTERN = re.compile(r"^[0-9a-fA-F]{40}(?:[0-9a-fA-F]{24})?$")

__all__ = ["build_document", "compare_revisions", "validate_request"]


def _revision(value: object) -> str | None:
    if not isinstance(value, str):
        return None
    candidate = value.strip()
    if not SHA_PATTERN.fullmatch(candidate):
        return None
    return candidate.lower()


def _target(value: object) -> object:
    if isinstance(value, Mapping):
        return value.get("target")
    return value


def compare_revisions(
    installed: Mapping[str, object], remote: Mapping[str, object]
) -> dict[str, dict[str, bool | str]]:
    """Compare installed and target SHAs without guessing unknown values.

    This intentionally copies the small comparison rule from update_service:
    the host runner must not import from pib_api/, which would drag Flask and
    its dependencies onto the host.
    """
    result: dict[str, dict[str, bool | str]] = {}
    for repository in sorted(set(installed) | set(remote)):
        current = _revision(installed.get(repository))
        target = _revision(_target(remote.get(repository)))
        result[repository] = {
            "installed": current or UNKNOWN,
            "target": target or UNKNOWN,
            "updateAvailable": (
                current != target
                if current is not None and target is not None
                else UNKNOWN
            ),
        }
    return result


def build_document(
    installed: Mapping[str, object],
    remote: Mapping[str, object],
    checked_at: object,
    *,
    check_id: object = None,
    channel: object = None,
    state: object = None,
    previous: object = None,
    releases: object = None,
) -> dict[str, Any]:
    """Build the availability protocol document.

    A release-channel document does not treat an unequal branch SHA as a newer
    published system release. That answer is the paired tag list, added later
    by ``update_releases``.
    """
    if not isinstance(checked_at, str) or not checked_at.strip():
        raise ValueError("invalid checked_at")
    repositories = compare_revisions(installed, remote)
    for repository, value in remote.items():
        if not isinstance(value, Mapping):
            continue
        error = value.get("error")
        if repository in repositories and isinstance(error, str) and error.strip():
            repositories[repository]["error"] = error.strip()
    if channel == "release":
        for repository in repositories.values():
            repository["branchTarget"] = repository.get("target")
            repository["updateAvailable"] = UNKNOWN
    document: dict[str, Any] = {
        "schemaVersion": 1,
        "checkedAt": checked_at.strip(),
        "repositories": repositories,
    }
    if isinstance(state, str) and state.strip():
        document["state"] = state.strip()
    if isinstance(channel, str) and channel.strip():
        document["channel"] = channel.strip()
    if isinstance(check_id, str) and check_id.strip():
        document["checkId"] = check_id.strip()
    if isinstance(previous, Mapping):
        retained = {key: value for key, value in previous.items() if key != "previous"}
        document["previous"] = retained
    if isinstance(releases, Mapping):
        for key in ("releases", "incomplete", "excluded", "latestInstallable", "error"):
            if key in releases:
                document[key] = releases[key]
    return document


def validate_request(document: object) -> dict[str, Any]:
    """Strictly validate a check request and name the offending field."""
    if not isinstance(document, dict):
        raise ValueError("invalid request field: document")
    required = {
        "schemaVersion": int,
        "requestedAt": str,
        "actor": str,
        "channel": str,
    }
    for name, expected_type in required.items():
        if name not in document:
            raise ValueError(f"invalid request field: {name} (missing)")
        value = document[name]
        if type(value) is not expected_type or (
            expected_type is str and not value.strip()
        ):
            raise ValueError(f"invalid request field: {name}")
    if document["schemaVersion"] != 1:
        raise ValueError("invalid request field: schemaVersion")
    if document["channel"] not in ALLOWED_CHANNELS:
        raise ValueError("invalid request field: channel")
    validated = {
        "schemaVersion": 1,
        "requestedAt": document["requestedAt"].strip(),
        "actor": document["actor"].strip(),
        "channel": document["channel"],
    }
    if "checkId" in document:
        check_id = document["checkId"]
        if (
            type(check_id) is not str
            or not re.fullmatch(r"[0-9a-fA-F-]{36}", check_id.strip())
        ):
            raise ValueError("invalid request field: checkId")
        validated["checkId"] = check_id.strip()
    return validated


def _read_json(path: Path) -> object:
    with path.open(encoding="utf-8") as source:
        return json.load(source)


def _atomic_write(path: Path, document: Mapping[str, Any]) -> None:
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", dir=path.parent
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as output:
            json.dump(document, output, sort_keys=True)
            output.write("\n")
            output.flush()
            os.fsync(output.fileno())
        os.chmod(temporary_name, 0o660)
        os.replace(temporary_name, path)
    except BaseException:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise


def commit_available(spec: Mapping[str, Any]) -> dict[str, Any]:
    """Write the check identity into the availability document."""
    previous = spec.get("previous")
    if not isinstance(previous, Mapping):
        previous = None
    releases = spec.get("releases")
    if not isinstance(releases, Mapping):
        releases = None
    return build_document(
        spec.get("installed") if isinstance(spec.get("installed"), Mapping) else {},
        spec.get("remote") if isinstance(spec.get("remote"), Mapping) else {},
        spec.get("checkedAt"),
        check_id=spec.get("checkId"),
        channel=spec.get("channel"),
        state=spec.get("state") or "completed",
        previous=previous,
        releases=releases,
    )


def main(argv: list[str] | None = None) -> int:
    arguments = list(sys.argv[1:] if argv is None else argv)
    if len(arguments) == 2 and arguments[0] == "validate-request":
        request = validate_request(_read_json(Path(arguments[1])))
        print("CHANNEL=" + shlex.quote(request["channel"]))
        print("BRANCH=" + shlex.quote(CHANNEL_BRANCHES[request["channel"]]))
        print("CHECK_ID=" + shlex.quote(request.get("checkId", "")))
        return 0
    if len(arguments) == 11 and arguments[0] == "write":
        path = Path(arguments[1])
        checked_at = arguments[2]
        installed = {
            arguments[3]: arguments[4],
            arguments[7]: arguments[8],
        }
        remote = {
            arguments[3]: {"target": arguments[5], "error": arguments[6]},
            arguments[7]: {"target": arguments[9], "error": arguments[10]},
        }
        _atomic_write(path, build_document(installed, remote, checked_at))
        return 0
    if len(arguments) == 3 and arguments[0] == "commit":
        spec = _read_json(Path(arguments[1]))
        if not isinstance(spec, dict):
            sys.stderr.write("commit spec must be a JSON object\n")
            return 2
        _atomic_write(Path(arguments[2]), commit_available(spec))
        return 0
    sys.stderr.write(
        "usage: update_check.py validate-request REQUEST_FILE\n"
        "       update_check.py write AVAILABLE_FILE CHECKED_AT "
        "REPOSITORY INSTALLED TARGET ERROR [REPOSITORY INSTALLED TARGET ERROR]\n"
        "       update_check.py commit SPEC_FILE AVAILABLE_FILE\n"
    )
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
