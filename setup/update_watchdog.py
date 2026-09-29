"""Pure watchdog timeout decisions for the host-side update runner.

The privileged helper deliberately duplicates the final validation rule instead of
importing this module: the checkout is writable by ``pib`` and must never become part
of a root trust boundary.
"""

from __future__ import annotations

import re
import sys
from decimal import Decimal, InvalidOperation

MAX_TARGET_US = 1_800_000_000

_DURATION_PART = re.compile(r"(?P<value>\d+(?:\.\d+)?)\s*(?P<unit>us|ms|s|min|h|d)")
_UNIT_US = {
    "us": Decimal(1),
    "ms": Decimal(1_000),
    "s": Decimal(1_000_000),
    "min": Decimal(60_000_000),
    "h": Decimal(3_600_000_000),
    "d": Decimal(86_400_000_000),
}

__all__ = [
    "MAX_TARGET_US",
    "parse_systemd_duration_us",
    "target_timeout_us",
    "validate_target_document",
]


def _non_negative_integer(value: object, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return value


def target_timeout_us(current_us: int, requested_us: int) -> int:
    """Return a safe update timeout without enabling or shortening a watchdog."""
    current = _non_negative_integer(current_us, "current_us")
    requested = _non_negative_integer(requested_us, "requested_us")
    if current == 0:
        return 0
    return max(current, min(requested, MAX_TARGET_US))


def validate_target_document(document: str | bytes) -> int:
    """Validate the complete integer document consumed by the root helper."""
    if isinstance(document, bytes):
        try:
            document = document.decode("ascii")
        except UnicodeDecodeError as error:
            raise ValueError("watchdog target must be ASCII") from error
    if not isinstance(document, str) or re.fullmatch(r"[0-9]+\n*", document) is None:
        raise ValueError("watchdog target must contain one non-negative integer")
    value = int(document)
    if value > MAX_TARGET_US:
        raise ValueError(f"watchdog target exceeds {MAX_TARGET_US}")
    return value


def parse_systemd_duration_us(value: str) -> int:
    """Parse the duration format printed by ``systemctl show``."""
    text = value.strip()
    if text == "0":
        return 0
    position = 0
    total = Decimal(0)
    matched = False
    while position < len(text):
        match = _DURATION_PART.match(text, position)
        if match is None:
            raise ValueError(f"invalid systemd duration: {value!r}")
        try:
            total += Decimal(match.group("value")) * _UNIT_US[match.group("unit")]
        except InvalidOperation as error:
            raise ValueError(f"invalid systemd duration: {value!r}") from error
        matched = True
        position = match.end()
        while position < len(text) and text[position].isspace():
            position += 1
    if not matched or total != total.to_integral_value():
        raise ValueError(
            f"duration is not an integer number of microseconds: {value!r}"
        )
    return int(total)


def main(argv: list[str] | None = None) -> int:
    arguments = list(sys.argv[1:] if argv is None else argv)
    try:
        if len(arguments) == 2 and arguments[0] == "parse":
            print(parse_systemd_duration_us(arguments[1]))
        elif len(arguments) == 3 and arguments[0] == "target":
            current = parse_systemd_duration_us(arguments[1])
            requested = _non_negative_integer(int(arguments[2]), "requested_us")
            print(target_timeout_us(current, requested))
        else:
            sys.stderr.write(
                "usage: update_watchdog.py parse CURRENT | "
                "target CURRENT REQUESTED_US\n"
            )
            return 2
    except (TypeError, ValueError) as error:
        sys.stderr.write(f"{error}\n")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
