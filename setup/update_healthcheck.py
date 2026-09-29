"""Decide whether an update made the running stack worse (decision D12, PR-1794).

The health gate compares the services that ran BEFORE the update with the ones running
afterwards:

* a service that ran before and is gone afterwards is a **regression** and fails the
  update (the rollback path already exists for that),
* a service that was already not running before the update is **reported** instead of
  blocking, because the strict rule deadlocks the robot: the damage blocks the very
  update that would fix it (observed live - an invalid bricklet UID crash-looped
  ros-motors, and every update then rolled back),
* if nothing ran before the update the strict rule applies, otherwise a single
  running container would make the gate meaningless.

The empty lists are therefore meaningful: an empty ``before`` means "nothing was
running", not "no information" - the caller passes the snapshot it took before the
update.

Deliberately dependency free so the rule stays unit-testable on a host without docker.
"""

from __future__ import annotations

import os
import sys
from typing import Iterable

__all__ = ["classify", "format_output", "main", "split_services"]


def split_services(value: str | Iterable[str] | None) -> set[str]:
    """Accept a newline/comma separated string or an iterable of service names."""
    if value is None:
        return set()
    if isinstance(value, str):
        parts = value.replace(",", "\n").splitlines()
    else:
        parts = [str(part) for part in value]
    return {part.strip() for part in parts if part.strip()}


def classify(
    expected: str | Iterable[str] | None,
    before: str | Iterable[str] | None,
    after: str | Iterable[str] | None,
) -> dict[str, object]:
    """Compare the stacks before and after the update.

    ``expected`` are the services the compose invocation defines, ``before``/``after``
    the services that were running at those two moments.
    """
    expected_set = split_services(expected)
    before_set = split_services(before)
    after_set = split_services(after)
    strict = not before_set
    reference = expected_set if strict else before_set
    return {
        "regressions": sorted(reference - after_set),
        "unhealthy": sorted(expected_set - after_set),
        "strict": strict,
    }


def format_output(result: dict[str, object]) -> str:
    regressions = ",".join(result["regressions"])  # type: ignore[arg-type]
    unhealthy = ",".join(result["unhealthy"])  # type: ignore[arg-type]
    return (
        f"REGRESSIONS={regressions}\n"
        f"UNHEALTHY={unhealthy}\n"
        f"STRICT={'1' if result['strict'] else '0'}\n"
    )


def main(argv: list[str] | None = None) -> int:
    """Print the verdict; exit 1 when the update would be a regression.

    Either pass three lists as arguments (expected, before, after) or set
    HEALTH_EXPECTED / HEALTH_BEFORE / HEALTH_AFTER in the environment - the runner
    uses the environment because service names arrive as newline separated output.
    """
    arguments = list(sys.argv[1:] if argv is None else argv)
    if len(arguments) == 3:
        expected, before, after = arguments
    else:
        expected = os.environ.get("HEALTH_EXPECTED", "")
        before = os.environ.get("HEALTH_BEFORE", "")
        after = os.environ.get("HEALTH_AFTER", "")
    result = classify(expected, before, after)
    sys.stdout.write(format_output(result))
    return 1 if result["regressions"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
