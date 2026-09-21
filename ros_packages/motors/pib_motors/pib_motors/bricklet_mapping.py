"""Decide which configured UIDs may become bricklet objects.

Deliberately free of ``tinkerforge`` and ``requests`` imports: the motor nodes
must be able to reason about their UIDs on a host without the hardware stack,
and this rule has to stay unit-testable.

The pattern mirrors ``validate_uid`` in
``pib_api/flask/service/hardware_config_service.py``; pib-api and the ROS
packages are separate deployment units and cannot share a module.
"""

from __future__ import annotations

import re
from typing import Any, Iterable, List, NamedTuple

# Tinkerforge UIDs are Base58, so '0', 'O', 'I' and 'l' are not UID characters.
UID_PATTERN = re.compile(r"^[1-9A-HJ-NP-Za-km-z]{1,6}$")
UID_FORBIDDEN_CHARACTERS = "0OIl"
UID_RULE_DESCRIPTION = (
    "expected Base58, max 6 characters; "
    "'0', 'O', 'I' and 'l' are not valid UID characters"
)


class SkippedUid(NamedTuple):
    """A configured UID that cannot be turned into a bricklet object."""

    uid: str
    reason: str


class BrickletUids(NamedTuple):
    valid: List[str]
    skipped: List[SkippedUid]


def uid_rejection_reason(value: Any) -> str | None:
    """Return why ``value`` is not a usable UID, or None if it is usable.

    An empty value means "not configured" and is usable in the sense that it is
    not an error - callers drop it via :func:`select_bricklet_uids`.
    """
    if value is None:
        return None
    if not isinstance(value, str):
        return f"expected a string, got {type(value).__name__}"
    if not value.strip():
        return None
    if not UID_PATTERN.fullmatch(value):
        return f"invalid format ({UID_RULE_DESCRIPTION})"
    return None


def select_bricklet_uids(values: Iterable[Any]) -> BrickletUids:
    """Split configured UIDs into the usable ones and the ones to report.

    Unconfigured (empty or missing) entries are dropped without a complaint;
    everything else that cannot be a UID lands in ``skipped`` with a reason, so
    the caller can log it instead of dying on it. Usable UIDs are passed through
    unchanged, because they double as the lookup key for the stored address.
    """
    valid: List[str] = []
    skipped: List[SkippedUid] = []
    for value in values:
        reason = uid_rejection_reason(value)
        if reason is not None:
            skipped.append(SkippedUid(str(value), reason))
        elif isinstance(value, str) and value.strip():
            valid.append(value)
    return BrickletUids(valid=valid, skipped=skipped)
