"""Compare an installed pib-sdk version against the repo floor (host runner).

Dependency-free: runs on the robot host next to setup/update_runner.sh.
"""

from __future__ import annotations

import re
import sys

__all__ = ["meets_floor", "parse_version", "verify"]

_VERSION = re.compile(r"^(\d+(?:\.\d+)*)$")


def parse_version(value: str) -> tuple[int, ...] | None:
    if not isinstance(value, str):
        return None
    match = _VERSION.match(value.strip())
    if not match:
        return None
    return tuple(int(part) for part in match.group(1).split("."))


def meets_floor(installed: str, required: str) -> tuple[bool, str]:
    """Return (ok, detail). ok is True when installed >= required."""
    installed_parts = parse_version(installed)
    required_parts = parse_version(required)
    if required_parts is None:
        return False, f"required version is not parseable: {required!r}"
    if installed_parts is None:
        return (
            False,
            f"installed version is not parseable: {installed!r} (required>={required})",
        )
    if installed_parts >= required_parts:
        return True, ""
    return (
        False,
        f"installed={installed.strip()} is below required>={required.strip()}",
    )


def verify(installed: str, required: str) -> tuple[bool, str]:
    """Alias for meets_floor; kept for tests and the runner CLI."""
    return meets_floor(installed, required)


def main(argv: list[str] | None = None) -> int:
    arguments = list(sys.argv[1:] if argv is None else argv)
    if len(arguments) != 2:
        sys.stderr.write("usage: pib_sdk_verify.py <installed> <required>\n")
        return 2
    ok, detail = verify(arguments[0], arguments[1])
    if ok:
        return 0
    sys.stderr.write(detail + "\n")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
