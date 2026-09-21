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
) -> dict[str, Any]:
    """Build the availability protocol document."""
    if not isinstance(checked_at, str) or not checked_at.strip():
        raise ValueError("invalid checked_at")
    repositories = compare_revisions(installed, remote)
    for repository, value in remote.items():
        if not isinstance(value, Mapping):
            continue
        error = value.get("error")
        if repository in repositories and isinstance(error, str) and error.strip():
            repositories[repository]["error"] = error.strip()
    return {
        "schemaVersion": 1,
        "checkedAt": checked_at.strip(),
        "repositories": repositories,
    }


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
    return {
        "schemaVersion": 1,
        "requestedAt": document["requestedAt"].strip(),
        "actor": document["actor"].strip(),
        "channel": document["channel"],
    }


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


def main(argv: list[str] | None = None) -> int:
    arguments = list(sys.argv[1:] if argv is None else argv)
    if len(arguments) == 2 and arguments[0] == "validate-request":
        request = validate_request(_read_json(Path(arguments[1])))
        print("CHANNEL=" + shlex.quote(request["channel"]))
        print("BRANCH=" + shlex.quote(CHANNEL_BRANCHES[request["channel"]]))
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
    sys.stderr.write(
        "usage: update_check.py validate-request REQUEST_FILE\n"
        "       update_check.py write AVAILABLE_FILE CHECKED_AT "
        "REPOSITORY INSTALLED TARGET ERROR [REPOSITORY INSTALLED TARGET ERROR]\n"
    )
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
