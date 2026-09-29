"""Read predecessor status for the host-side update runner."""

from __future__ import annotations

import json
import math
import shlex
import sys
from datetime import datetime, timezone
from pathlib import Path

TERMINAL_STATES = frozenset({"done", "failed", "rolled_back", "cancelled"})

__all__ = [
    "TERMINAL_STATES",
    "exceeded_attempt_limit",
    "next_attempt",
    "predecessor_interrupted",
    "retry_delay_remaining",
]


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


def exceeded_attempt_limit(document: object, job_id: str, max_attempts: int) -> bool:
    """Return whether the next attempt of this job must not start."""
    if isinstance(max_attempts, bool) or not isinstance(max_attempts, int):
        return False
    if max_attempts < 1:
        return False
    if not isinstance(document, dict) or document.get("jobId") != job_id:
        return False
    return next_attempt(document, job_id) > max_attempts


def _parse_datetime(value: object) -> datetime | None:
    if isinstance(value, datetime):
        parsed = value
    elif isinstance(value, str) and value:
        text = value[:-1] + "+00:00" if value.endswith("Z") else value
        try:
            parsed = datetime.fromisoformat(text)
        except ValueError:
            return None
    else:
        return None
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=timezone.utc)
    return parsed.astimezone(timezone.utc)


def retry_delay_remaining(now: object, updated_at: object, delay_seconds: int) -> int:
    """Return seconds still to wait before retrying; never negative."""
    if isinstance(delay_seconds, bool) or not isinstance(delay_seconds, int):
        return 0
    if delay_seconds <= 0:
        return 0
    parsed_now = _parse_datetime(now)
    parsed_updated = _parse_datetime(updated_at)
    if parsed_now is None or parsed_updated is None:
        return 0
    remaining = delay_seconds - (parsed_now - parsed_updated).total_seconds()
    if remaining <= 0:
        return 0
    return math.ceil(remaining)


def _read_document(path: Path) -> object:
    try:
        with path.open(encoding="utf-8") as source:
            return json.load(source)
    except (OSError, ValueError):
        return None


def _parse_int(value: str) -> int:
    try:
        return int(value)
    except ValueError:
        return 0


def main(argv: list[str] | None = None) -> int:
    arguments = list(sys.argv[1:] if argv is None else argv)
    if arguments and arguments[0] == "exceeded_attempt_limit":
        if len(arguments) != 4:
            sys.stderr.write(
                "usage: update_status_reader.py exceeded_attempt_limit "
                "STATUS_FILE JOB_ID MAX_ATTEMPTS\n"
            )
            return 2
        document = _read_document(Path(arguments[1]))
        exceeded = exceeded_attempt_limit(
            document, arguments[2], _parse_int(arguments[3])
        )
        print("true" if exceeded else "false")
        return 0
    if arguments and arguments[0] == "retry_delay_remaining":
        if len(arguments) != 3:
            sys.stderr.write(
                "usage: update_status_reader.py retry_delay_remaining "
                "STATUS_FILE DELAY_SECONDS\n"
            )
            return 2
        document = _read_document(Path(arguments[1]))
        updated_at = document.get("updatedAt") if isinstance(document, dict) else None
        remaining = retry_delay_remaining(
            datetime.now(timezone.utc),
            updated_at,
            _parse_int(arguments[2]),
        )
        print(remaining)
        return 0
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
