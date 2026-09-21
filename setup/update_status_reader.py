"""Read predecessor status for the host-side update runner."""

from __future__ import annotations

import json
import shlex
import sys
from pathlib import Path

TERMINAL_STATES = frozenset({"done", "failed", "rolled_back", "cancelled"})

__all__ = ["TERMINAL_STATES", "next_attempt", "predecessor_interrupted"]


def predecessor_interrupted(document: object) -> bool:
    """Return whether a valid status document describes an unfinished run."""
    if not isinstance(document, dict):
        return False
    state = document.get("state")
    return isinstance(state, str) and bool(state) and state not in TERMINAL_STATES


def next_attempt(document: object, job_id: str | None = None) -> int:
    """Return the next positive attempt number, scoped to one job when requested."""
    if not isinstance(document, dict):
        return 1
    if job_id is not None and document.get("jobId") != job_id:
        return 1
    attempt = document.get("attempt", 1)
    if isinstance(attempt, bool) or not isinstance(attempt, int) or attempt < 1:
        return 1
    return attempt + 1


def _read_document(path: Path) -> object:
    try:
        with path.open(encoding="utf-8") as source:
            return json.load(source)
    except (OSError, ValueError):
        return None


def main(argv: list[str] | None = None) -> int:
    arguments = list(sys.argv[1:] if argv is None else argv)
    if len(arguments) != 2:
        sys.stderr.write("usage: update_status_reader.py STATUS_FILE JOB_ID\n")
        return 2
    document = _read_document(Path(arguments[0]))
    interrupted = predecessor_interrupted(document)
    attempt = next_attempt(document, arguments[1])
    state = (
        document.get("state", "unknown") if isinstance(document, dict) else "unknown"
    )
    if not isinstance(state, str) or not state:
        state = "unknown"
    print(f"PREDECESSOR_INTERRUPTED={'true' if interrupted else 'false'}")
    print(f"ATTEMPT={attempt}")
    print(f"PREDECESSOR_STATE={shlex.quote(state)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
