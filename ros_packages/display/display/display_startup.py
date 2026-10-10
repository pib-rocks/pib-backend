"""Start the native display only on a usable host Wayland session.

PyGObject calls Gtk.init_check() once, when the Gtk namespace is imported, and GTK 4
marks itself initialised before it opens the display. A process whose first attempt
failed therefore cannot try again; every Gtk.Window it creates raises "Gtk couldn't be
initialized". This module imports nothing from GTK. It waits for a compositor that
accepts connections and only then imports the renderer. A renderer that does not come
up ends the process with a non-zero status, so ROS launch and Docker see a failure.
"""

from __future__ import annotations

import os
import socket
import stat
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Mapping, Optional

# EX_TEMPFAIL: the host session is missing, a later start may succeed.
EXIT_WAYLAND_UNAVAILABLE = 75
EXIT_RENDERER_FAILED = 1

DEFAULT_WAIT_SECONDS = 600.0
WAIT_SECONDS_VARIABLE = "PIB_DISPLAY_WAYLAND_WAIT_SECONDS"

AVAILABLE = "available"
UNCONFIGURED = "unconfigured"
ABSENT = "absent"
NOT_A_SOCKET = "not-a-socket"
REFUSED = "refused"
DENIED = "denied"
UNUSABLE = "unusable"


@dataclass(frozen=True)
class WaylandProbe:
    state: str
    path: Optional[Path]
    detail: str

    @property
    def usable(self) -> bool:
        return self.state == AVAILABLE


def log(message: str) -> None:
    print(f"[display] {message}", file=sys.stderr, flush=True)


def wayland_socket_path(environ: Mapping[str, str]) -> Optional[Path]:
    """The socket wl_display_connect(NULL) opens for this environment."""
    name = environ.get("WAYLAND_DISPLAY") or "wayland-0"
    if os.path.isabs(name):
        return Path(name)
    runtime_dir = environ.get("XDG_RUNTIME_DIR")
    if not runtime_dir:
        return None
    return Path(runtime_dir) / name


def probe_wayland(environ: Mapping[str, str], timeout: float = 1.0) -> WaylandProbe:
    """Connect to the compositor socket once; a file that only exists is not enough."""
    path = wayland_socket_path(environ)
    if path is None:
        return WaylandProbe(
            UNCONFIGURED,
            None,
            "XDG_RUNTIME_DIR is not set and WAYLAND_DISPLAY is not an absolute path",
        )
    try:
        mode = os.stat(path).st_mode
    except FileNotFoundError:
        if not path.parent.is_dir():
            return WaylandProbe(
                ABSENT,
                path,
                f"{path.parent} does not exist: no host session for this user yet",
            )
        return WaylandProbe(
            ABSENT,
            path,
            f"{path} does not exist: the host graphical session (LightDM/labwc) "
            "has not started a compositor",
        )
    except PermissionError as exc:
        return WaylandProbe(DENIED, path, _denied(path, exc))
    except OSError as exc:
        return WaylandProbe(UNUSABLE, path, f"{path} cannot be inspected: {exc}")
    if not stat.S_ISSOCK(mode):
        return WaylandProbe(NOT_A_SOCKET, path, f"{path} exists but is not a socket")

    client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    client.settimeout(timeout)
    try:
        client.connect(str(path))
    except PermissionError as exc:
        return WaylandProbe(DENIED, path, _denied(path, exc))
    except ConnectionRefusedError:
        return WaylandProbe(
            REFUSED,
            path,
            f"{path} exists but no compositor accepts connections (stale socket)",
        )
    except OSError as exc:
        return WaylandProbe(UNUSABLE, path, f"{path} cannot be connected: {exc}")
    finally:
        client.close()
    return WaylandProbe(AVAILABLE, path, f"{path} accepts connections")


def _denied(path: Path, exc: OSError) -> str:
    return f"{path} is not accessible to uid {os.getuid()}: {exc.strerror}"


def wait_seconds(environ: Mapping[str, str], log: Callable[[str], None]) -> float:
    value = environ.get(WAIT_SECONDS_VARIABLE)
    if value is None:
        return DEFAULT_WAIT_SECONDS
    try:
        seconds = float(value)
    except ValueError:
        seconds = -1.0
    if seconds < 0:
        log(
            f"{WAIT_SECONDS_VARIABLE}={value!r} is not a number of seconds; "
            f"using {DEFAULT_WAIT_SECONDS:.0f}"
        )
        return DEFAULT_WAIT_SECONDS
    return seconds


def wait_for_wayland(
    environ: Mapping[str, str],
    *,
    max_wait: float,
    log: Callable[[str], None],
    probe: Callable[[Mapping[str, str]], WaylandProbe] = probe_wayland,
    sleep: Callable[[float], None] = time.sleep,
    monotonic: Callable[[], float] = time.monotonic,
    first_delay: float = 1.0,
    max_delay: float = 30.0,
    remind_every: float = 300.0,
) -> WaylandProbe:
    """Probe until the compositor accepts connections or max_wait has passed.

    The interval doubles up to max_delay. A change of the failure is logged at once,
    an unchanged one every remind_every seconds.
    """
    start = monotonic()
    delay = first_delay
    reported: Optional[tuple[str, str]] = None
    reported_at = start
    while True:
        result = probe(environ)
        now = monotonic()
        elapsed = now - start
        if result.usable:
            if reported is not None:
                log(f"{result.detail}; became usable after {elapsed:.0f}s")
            return result
        current = (result.state, result.detail)
        if current != reported or now - reported_at >= remind_every:
            log(
                f"waiting for the host Wayland session ({elapsed:.0f}s): "
                f"{result.detail}"
            )
            reported = current
            reported_at = now
        if elapsed >= max_wait:
            log(
                f"host Wayland session not usable after {elapsed:.0f}s: {result.detail}"
            )
            return result
        sleep(min(delay, max_wait - elapsed))
        delay = min(delay * 2, max_delay)


class RendererReadiness:
    """Shared between the GTK main loop, which sets it, and the ROS node, which reports it."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._ready = False
        self._failure: Optional[str] = None

    def mark_ready(self) -> None:
        with self._lock:
            if self._failure is None:
                self._ready = True

    def mark_failed(self, reason: str) -> None:
        with self._lock:
            self._ready = False
            self._failure = reason

    @property
    def ready(self) -> bool:
        with self._lock:
            return self._ready

    @property
    def failure(self) -> Optional[str]:
        with self._lock:
            return self._failure


def renderer_exit_status(run_status: int, readiness: RendererReadiness) -> int:
    """A main loop that ends without a usable window is a failure, whatever it returned."""
    if readiness.failure is not None or not readiness.ready:
        return EXIT_RENDERER_FAILED
    return run_status


def main(args=None) -> int:
    # Before GTK is imported: no accidental X11 fallback.
    os.environ.setdefault("GDK_BACKEND", "wayland")

    result = wait_for_wayland(
        os.environ, max_wait=wait_seconds(os.environ, log), log=log
    )
    if not result.usable:
        log(
            f"display not started; exiting with status {EXIT_WAYLAND_UNAVAILABLE} "
            "so the container starts again and re-reads the runtime directory"
        )
        return EXIT_WAYLAND_UNAVAILABLE

    from display import display_wayland_gtk

    return display_wayland_gtk.run_renderer()
