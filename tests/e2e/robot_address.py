"""Robot address for the live pytest E2E suite.

Set ``PIB_ROBOT_URL`` to the robot (for example ``http://192.168.1.172``).
The API address is that value plus ``/api``; it is not configured separately.
When ``PIB_ROBOT_URL`` is unset, ``PIB_API_URL``, ``PIB_E2E_BASE_URL``,
``PIB_MODEL_E2E_HOST``, and ``PIB_MODEL_E2E_API_URL`` still work as fallbacks.
The run states which variable it used. If more than one is set, they must
name the same robot — the suite does not choose between them. With none set,
resolution fails instead of using ``http://localhost``.
"""

from __future__ import annotations

import os
from collections.abc import Mapping
from dataclasses import dataclass
from urllib.parse import urlparse

# Documented name first, then the older names kept as fallbacks.
ADDRESS_VARIABLES = (
    "PIB_ROBOT_URL",
    "PIB_API_URL",
    "PIB_E2E_BASE_URL",
    "PIB_MODEL_E2E_HOST",
    "PIB_MODEL_E2E_API_URL",
)

_API_VARIABLES = {"PIB_API_URL", "PIB_MODEL_E2E_API_URL"}
_HOST_VARIABLE = "PIB_MODEL_E2E_HOST"


class RobotAddressError(Exception):
    """The live suite has no single robot address it can use."""


@dataclass(frozen=True)
class ResolvedRobot:
    url: str
    sources: tuple[str, ...]

    def summary(self) -> str:
        if self.sources == ("PIB_ROBOT_URL",):
            return f"Live E2E robot address from PIB_ROBOT_URL: {self.url}"
        if "PIB_ROBOT_URL" in self.sources:
            others = ", ".join(name for name in self.sources if name != "PIB_ROBOT_URL")
            return (
                f"Live E2E robot address from PIB_ROBOT_URL: {self.url} "
                f"(same robot also set in {others})"
            )
        joined = ", ".join(self.sources)
        return (
            f"Live E2E robot address from {joined}: {self.url} "
            "(PIB_ROBOT_URL is unset; using the fallback)"
        )


@dataclass(frozen=True)
class _Candidate:
    name: str
    raw: str
    base_url: str
    identity: tuple[str, str, int, str]


def resolve(environ: Mapping[str, str] | None = None) -> ResolvedRobot:
    """Return the one robot base URL described by ``environ``."""
    source = os.environ if environ is None else environ
    candidates = [
        _candidate(name, raw)
        for name in ADDRESS_VARIABLES
        if (raw := _configured(source, name)) is not None
    ]
    if not candidates:
        raise RobotAddressError(
            "No live-robot address is configured. Set PIB_ROBOT_URL to the "
            "robot address, for example PIB_ROBOT_URL=http://192.168.1.172. "
            "The live E2E suite stops instead of trying http://localhost. "
            "Fallbacks accepted when they name that same robot: "
            "PIB_API_URL, PIB_E2E_BASE_URL, PIB_MODEL_E2E_HOST, "
            "PIB_MODEL_E2E_API_URL. The run states which variable it used."
        )
    identities = {item.identity for item in candidates}
    if len(identities) != 1:
        rendered = "; ".join(f"{item.name}={item.raw}" for item in candidates)
        raise RobotAddressError(
            "Live E2E robot address variables disagree, so none was used "
            f"({rendered}). Set PIB_ROBOT_URL to one address, or make every "
            "fallback describe that same robot. The suite will not choose "
            "between them."
        )
    return ResolvedRobot(
        url=candidates[0].base_url,
        sources=tuple(item.name for item in candidates),
    )


def robot_base_url(environ: Mapping[str, str] | None = None) -> str:
    """The robot UI address, or ``""`` when it cannot be resolved.

    Importing a live test must not fail collection when nothing is configured.
    The suite stops later with :class:`RobotAddressError`.
    """
    try:
        return resolve(environ).url
    except RobotAddressError:
        return ""


def api_url(environ: Mapping[str, str] | None = None) -> str:
    """API address derived from the robot address (``<robot>/api``)."""
    base = robot_base_url(environ)
    if not base:
        return ""
    return f"{base}/api"


def robot_hostname(environ: Mapping[str, str] | None = None) -> str:
    base = robot_base_url(environ)
    if not base:
        return ""
    return urlparse(base).hostname or ""


def _configured(environ: Mapping[str, str], name: str) -> str | None:
    raw = environ.get(name)
    if raw is None:
        return None
    value = raw.strip()
    if not value:
        return None
    return value


def _candidate(name: str, raw: str) -> _Candidate:
    base_url = _base_url(name, raw)
    return _Candidate(
        name=name,
        raw=raw,
        base_url=base_url,
        identity=_identity(base_url),
    )


def _base_url(name: str, raw: str) -> str:
    value = raw.strip().rstrip("/")
    if name == _HOST_VARIABLE and "://" not in value:
        value = f"http://{value}"
    if name in _API_VARIABLES:
        if not value.endswith("/api"):
            raise RobotAddressError(
                f"{name}={raw!r} is not an API address ending in /api. "
                "Set PIB_ROBOT_URL to the robot, for example "
                "PIB_ROBOT_URL=http://192.168.1.172. The API address is "
                "derived from that."
            )
        value = value[: -len("/api")].rstrip("/")
    _require_absolute_url(name, raw, value)
    if name == _HOST_VARIABLE and urlparse(value).path not in ("", "/"):
        raise RobotAddressError(
            f"{name}={raw!r} is not a robot host. Set PIB_ROBOT_URL to an "
            "absolute URL such as http://192.168.1.172."
        )
    return value


def _require_absolute_url(name: str, raw: str, value: str) -> None:
    parsed = urlparse(value)
    if (
        parsed.scheme not in {"http", "https"}
        or not parsed.hostname
        or parsed.username
        or parsed.password
        or parsed.query
        or parsed.fragment
    ):
        raise RobotAddressError(
            f"{name}={raw!r} is not a robot address. Set PIB_ROBOT_URL to an "
            "absolute URL such as http://192.168.1.172."
        )


def _identity(url: str) -> tuple[str, str, int, str]:
    parsed = urlparse(url)
    if parsed.port is None:
        port = 443 if parsed.scheme == "https" else 80
    else:
        port = parsed.port
    return (
        parsed.scheme.lower(),
        parsed.hostname.lower(),
        port,
        parsed.path.rstrip("/"),
    )
