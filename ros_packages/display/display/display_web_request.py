"""Request file for the host-side fullscreen browser.

The display container writes ``display-web.json`` and never removes it. The
host runner (``setup/display_web_runner.sh``) validates that file, writes
``display-web-status.json``, and removes the request. Two writers on the
request file would race, so this module only creates or replaces it.
"""

from __future__ import annotations

import json
import os
import re
import shlex
import sys
import tempfile
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from urllib.parse import urlparse

SCHEMA_VERSION = 1
REQUEST_FILENAME = "display-web.json"
STATUS_FILENAME = "display-web-status.json"
SURFACE_READY = "ready"
SURFACE_WEB = "web"
SURFACE_WEB_FAILED = "web-failed"
DEFAULT_ACK_SECONDS = 15.0

_REQUESTED_AT = re.compile(r"^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}Z$")
_DRAW_KINDS = {"text", "expression", "raw"}


def update_directory() -> Path:
    """Directory the container writes into. The compose file bind-mounts it."""
    return Path(os.environ.get("PIB_UPDATE_DIR", "/app/.update"))


def utc_now() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def validate_url(url: object) -> str:
    if type(url) is not str:
        raise ValueError("invalid request field: url")
    if not url or any(character.isspace() for character in url):
        raise ValueError("invalid request field: url")
    if url.startswith("-"):
        raise ValueError("invalid request field: url")
    parsed = urlparse(url)
    if parsed.scheme not in {"http", "https"} or not parsed.netloc:
        raise ValueError("invalid request field: url")
    return url


def validate_document(document: object) -> dict:
    if not isinstance(document, dict):
        raise ValueError("display-web.json must be a JSON object")
    version = document.get("schemaVersion")
    if type(version) is not int or version != SCHEMA_VERSION:
        raise ValueError("invalid request field: schemaVersion")
    action = document.get("action")
    if action not in {"open", "hide"}:
        raise ValueError("invalid request field: action")
    url = document.get("url")
    if type(url) is not str:
        raise ValueError("invalid request field: url")
    if action == "open":
        url = validate_url(url)
    elif url != "":
        raise ValueError("invalid request field: url")
    requested_at = document.get("requestedAt")
    if type(requested_at) is not str or _REQUESTED_AT.fullmatch(requested_at) is None:
        raise ValueError("invalid request field: requestedAt")
    return {
        "schemaVersion": SCHEMA_VERSION,
        "action": action,
        "url": url,
        "requestedAt": requested_at,
    }


def load_request(path: Path) -> dict:
    try:
        with path.open(encoding="utf-8") as source:
            document = json.load(source)
    except json.JSONDecodeError as exc:
        raise ValueError("display-web.json is not valid JSON") from exc
    return validate_document(document)


def _atomic_write(directory: Path, document: dict) -> None:
    """Write the request by mkstemp + rename. Never removes the request file."""
    descriptor, temporary = tempfile.mkstemp(prefix=".display-web.json.", dir=directory)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as output:
            json.dump(document, output, sort_keys=True)
            output.write("\n")
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, directory / REQUEST_FILENAME)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def write_open_request(
    directory: Path, url: str, requested_at: str | None = None
) -> dict:
    document = validate_document(
        {
            "schemaVersion": SCHEMA_VERSION,
            "action": "open",
            "url": url,
            "requestedAt": requested_at or utc_now(),
        }
    )
    _atomic_write(directory, document)
    return document


def write_hide_request(directory: Path, requested_at: str | None = None) -> dict:
    document = validate_document(
        {
            "schemaVersion": SCHEMA_VERSION,
            "action": "hide",
            "url": "",
            "requestedAt": requested_at or utc_now(),
        }
    )
    _atomic_write(directory, document)
    return document


def read_status(directory: Path) -> dict | None:
    path = directory / STATUS_FILENAME
    try:
        with path.open(encoding="utf-8") as source:
            document = json.load(source)
    except (OSError, json.JSONDecodeError):
        return None
    if not isinstance(document, dict):
        return None
    return document


def command_effect(web_surface_open: bool, kind: str) -> str:
    """How the GTK window should treat one display command.

    ``hide`` keeps its existing meaning (hide the GTK window only). While the
    web surface is open, face, text, and image commands are not drawn.
    """
    if kind == "web_open":
        return "web_open"
    if kind == "web_hide":
        return "web_hide"
    if kind == "hide":
        return "hide"
    if kind in _DRAW_KINDS:
        if web_surface_open:
            return "suppress"
        return "draw"
    return "ignore"


@dataclass(frozen=True)
class WebAcknowledgement:
    """What the node should do with one look at the host status file.

    ``suppression`` is ``hold`` (browser is the surface), ``release`` (GTK may
    draw again, but nothing is drawn until the next command), or ``None``.
    """

    surface: str | None = None
    suppression: str | None = None
    clear_pending: bool = False


def acknowledge_web_status(
    pending_action: str,
    pending_requested_at: str,
    status: dict | None,
    timed_out: bool,
    failure_reported: bool,
) -> WebAcknowledgement:
    if isinstance(status, dict) and status.get("requestedAt") == pending_requested_at:
        state = status.get("state")
        action = status.get("action")
        if state == "done" and action == "open":
            return WebAcknowledgement(SURFACE_WEB, "hold", True)
        if state == "done" and action == "hide":
            return WebAcknowledgement(SURFACE_READY, "release", True)
        if state == "failed":
            return WebAcknowledgement(SURFACE_WEB_FAILED, "release", True)
    if timed_out and not failure_reported:
        # No host answer. Release so the face cannot stay suppressed forever,
        # and keep the pending request so a late status can still take over.
        return WebAcknowledgement(SURFACE_WEB_FAILED, "release", False)
    return WebAcknowledgement()


def later_hide_releases(pending_requested_at: str, status: dict | None) -> bool:
    """A hide written after the open closed the password page.

    The pending request is the open. The host status for a later hide carries
    a newer timestamp, so the face can come back without waiting out the
    acknowledgement timeout.
    """
    if not isinstance(status, dict):
        return False
    if status.get("state") != "done" or status.get("action") != "hide":
        return False
    requested_at = status.get("requestedAt")
    if type(requested_at) is not str or type(pending_requested_at) is not str:
        return False
    return requested_at > pending_requested_at


def _emit(ok: bool, action: str, url: str, requested_at: str, message: str) -> None:
    print("OK=" + ("true" if ok else "false"))
    print("ACTION=" + shlex.quote(action))
    print("URL=" + shlex.quote(url))
    print("REQUESTED_AT=" + shlex.quote(requested_at))
    print("MESSAGE=" + shlex.quote(message))


def _peek_requested_at(path: Path) -> str:
    try:
        with path.open(encoding="utf-8") as source:
            document = json.load(source)
    except (OSError, json.JSONDecodeError):
        return ""
    if isinstance(document, dict) and type(document.get("requestedAt")) is str:
        return document["requestedAt"]
    return ""


def main(argv: list[str]) -> int:
    if len(argv) != 3 or argv[1] != "validate":
        print(
            "usage: display_web_request.py validate REQUEST_FILE",
            file=sys.stderr,
        )
        return 2
    path = Path(argv[2])
    try:
        document = load_request(path)
    except ValueError as exc:
        _emit(False, "unknown", "", _peek_requested_at(path), str(exc))
        return 0
    except OSError as exc:
        reason = exc.strerror or str(exc)
        _emit(False, "unknown", "", "", f"could not read display-web.json: {reason}")
        return 0
    _emit(True, document["action"], document["url"], document["requestedAt"], "")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
