"""Host Wayland prerequisite of the native display (PR-1980).

On the affected Pi 5 the host had no compositor and no wayland-0 socket. GTK failed in
do_activate, the process still exited 0 and Docker restarted it about once a minute.
display_startup checks the socket before GTK is imported, waits with a capped backoff,
and turns a failed GTK start into a non-zero exit status.

The probes use real AF_UNIX sockets in tmp_path; only the clock is faked.
"""

from __future__ import annotations

import os
import shutil
import socket
import sys
import tempfile
import types
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
DISPLAY_ROOT = REPO_ROOT / "ros_packages" / "display"
sys.path.insert(0, str(DISPLAY_ROOT))

import display.display_startup as startup  # noqa: E402


def test_module_resolves_to_this_worktree():
    assert Path(startup.__file__).resolve() == (
        DISPLAY_ROOT / "display" / "display_startup.py"
    )


def test_the_module_does_not_import_gtk():
    text = Path(startup.__file__).read_text(encoding="utf-8")
    assert "gi.repository" not in text
    assert "import gi" not in text
    assert "rclpy" not in text


# ---- socket path ---------------------------------------------------------------------------


def test_the_socket_path_follows_libwayland():
    assert startup.wayland_socket_path(
        {"XDG_RUNTIME_DIR": "/run/user/1000", "WAYLAND_DISPLAY": "wayland-1"}
    ) == Path("/run/user/1000/wayland-1")
    assert startup.wayland_socket_path({"XDG_RUNTIME_DIR": "/run/user/1000"}) == Path(
        "/run/user/1000/wayland-0"
    )
    assert startup.wayland_socket_path(
        {"WAYLAND_DISPLAY": "/tmp/compositor.sock"}
    ) == Path("/tmp/compositor.sock")
    assert startup.wayland_socket_path({"WAYLAND_DISPLAY": "wayland-0"}) is None


# ---- probe ---------------------------------------------------------------------------------


def _env(runtime: Path, name: str = "wayland-0") -> dict[str, str]:
    return {"XDG_RUNTIME_DIR": str(runtime), "WAYLAND_DISPLAY": name}


def _listener(path: Path) -> socket.socket:
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(str(path))
    server.listen(1)
    return server


@pytest.fixture
def runtime():
    # AF_UNIX paths are limited to 108 bytes, which pytest's tmp_path can exceed.
    directory = Path(tempfile.mkdtemp(prefix="pib-wl-", dir="/tmp"))
    yield directory
    shutil.rmtree(directory, ignore_errors=True)


def test_no_runtime_configuration_is_reported_as_such():
    result = startup.probe_wayland({"WAYLAND_DISPLAY": "wayland-0"})
    assert result.state == startup.UNCONFIGURED
    assert not result.usable
    assert "XDG_RUNTIME_DIR" in result.detail


def test_an_absent_socket_names_the_path_and_the_missing_session(runtime: Path):
    result = startup.probe_wayland(_env(runtime))

    assert result.state == startup.ABSENT
    assert not result.usable
    assert str(runtime / "wayland-0") in result.detail
    assert "session" in result.detail


def test_an_absent_runtime_directory_is_named(tmp_path: Path):
    result = startup.probe_wayland(_env(tmp_path / "missing"))

    assert result.state == startup.ABSENT
    assert str(tmp_path / "missing") in result.detail


def test_a_file_that_is_not_a_socket_is_unusable(runtime: Path):
    (runtime / "wayland-0").write_text("", encoding="utf-8")

    result = startup.probe_wayland(_env(runtime))

    assert result.state == startup.NOT_A_SOCKET
    assert not result.usable


def test_a_stale_socket_without_compositor_is_unusable(runtime: Path):
    stale = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    stale.bind(str(runtime / "wayland-0"))
    stale.close()
    assert (runtime / "wayland-0").is_socket()

    result = startup.probe_wayland(_env(runtime))

    assert result.state == startup.REFUSED
    assert not result.usable
    assert "compositor" in result.detail


@pytest.mark.skipif(os.geteuid() == 0, reason="root ignores socket permissions")
def test_a_socket_the_user_may_not_open_is_unusable(runtime: Path):
    server = _listener(runtime / "wayland-0")
    try:
        os.chmod(runtime / "wayland-0", 0)
        result = startup.probe_wayland(_env(runtime))
    finally:
        server.close()

    assert result.state == startup.DENIED
    assert not result.usable
    assert str(os.getuid()) in result.detail


def test_a_listening_compositor_socket_is_usable(runtime: Path):
    server = _listener(runtime / "wayland-0")
    try:
        result = startup.probe_wayland(_env(runtime))
    finally:
        server.close()

    assert result.state == startup.AVAILABLE
    assert result.usable
    assert result.path == runtime / "wayland-0"


# ---- waiting -------------------------------------------------------------------------------


class Clock:
    def __init__(self, on_sleep=None):
        self.now = 0.0
        self.sleeps: list[float] = []
        self.on_sleep = on_sleep

    def monotonic(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.sleeps.append(seconds)
        self.now += seconds
        if self.on_sleep is not None:
            self.on_sleep(len(self.sleeps))


def test_a_session_that_appears_later_is_picked_up_without_a_restart(runtime: Path):
    servers = []

    def compositor_starts(count: int) -> None:
        if count == 3:
            servers.append(_listener(runtime / "wayland-0"))

    clock = Clock(compositor_starts)
    messages: list[str] = []
    try:
        result = startup.wait_for_wayland(
            _env(runtime),
            max_wait=600,
            log=messages.append,
            sleep=clock.sleep,
            monotonic=clock.monotonic,
        )
    finally:
        for server in servers:
            server.close()

    assert result.usable
    assert clock.sleeps == [1.0, 2.0, 4.0]
    assert len([m for m in messages if "waiting for the host Wayland" in m]) == 1
    assert any("became usable after 7s" in m for m in messages), messages


def test_an_available_session_starts_without_waiting_or_noise(runtime: Path):
    server = _listener(runtime / "wayland-0")
    clock = Clock()
    messages: list[str] = []
    try:
        result = startup.wait_for_wayland(
            _env(runtime),
            max_wait=600,
            log=messages.append,
            sleep=clock.sleep,
            monotonic=clock.monotonic,
        )
    finally:
        server.close()

    assert result.usable
    assert clock.sleeps == []
    assert messages == []


def test_the_wait_is_bounded_and_the_backoff_is_capped(runtime: Path):
    clock = Clock()
    messages: list[str] = []

    result = startup.wait_for_wayland(
        _env(runtime),
        max_wait=100,
        log=messages.append,
        sleep=clock.sleep,
        monotonic=clock.monotonic,
        max_delay=30.0,
    )

    assert result.state == startup.ABSENT
    assert clock.sleeps == [1.0, 2.0, 4.0, 8.0, 16.0, 30.0, 30.0, 9.0]
    assert clock.now == 100
    assert any("not usable after 100s" in m for m in messages), messages


def test_every_change_of_the_failure_is_reported(runtime: Path):
    def stale_socket_appears(count: int) -> None:
        if count == 2:
            stale = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            stale.bind(str(runtime / "wayland-0"))
            stale.close()

    clock = Clock(stale_socket_appears)
    messages: list[str] = []

    startup.wait_for_wayland(
        _env(runtime),
        max_wait=10,
        log=messages.append,
        sleep=clock.sleep,
        monotonic=clock.monotonic,
    )

    waiting = [m for m in messages if "waiting for the host Wayland" in m]
    assert len(waiting) == 2, messages
    assert "does not exist" in waiting[0]
    assert "compositor" in waiting[1]


def test_a_long_wait_keeps_reminding(runtime: Path):
    clock = Clock()
    messages: list[str] = []

    startup.wait_for_wayland(
        _env(runtime),
        max_wait=700,
        log=messages.append,
        sleep=clock.sleep,
        monotonic=clock.monotonic,
        remind_every=300.0,
    )

    waiting = [m for m in messages if "waiting for the host Wayland" in m]
    assert len(waiting) == 3, messages


def test_the_wait_budget_comes_from_the_environment():
    messages: list[str] = []
    assert startup.wait_seconds({}, messages.append) == startup.DEFAULT_WAIT_SECONDS
    assert startup.wait_seconds({"PIB_DISPLAY_WAYLAND_WAIT_SECONDS": "30"}, print) == 30
    assert (
        startup.wait_seconds(
            {"PIB_DISPLAY_WAYLAND_WAIT_SECONDS": "soon"}, messages.append
        )
        == startup.DEFAULT_WAIT_SECONDS
    )
    assert (
        startup.wait_seconds(
            {"PIB_DISPLAY_WAYLAND_WAIT_SECONDS": "-5"}, messages.append
        )
        == startup.DEFAULT_WAIT_SECONDS
    )
    assert len(messages) == 2


# ---- readiness and exit status -------------------------------------------------------------


def test_a_renderer_that_never_became_ready_is_a_failure():
    readiness = startup.RendererReadiness()

    assert not readiness.ready
    assert startup.renderer_exit_status(0, readiness) == startup.EXIT_RENDERER_FAILED


def test_a_failed_renderer_is_a_failure_even_with_a_clean_main_loop():
    readiness = startup.RendererReadiness()
    readiness.mark_failed("Gtk couldn't be initialized")

    assert readiness.failure == "Gtk couldn't be initialized"
    assert not readiness.ready
    assert startup.renderer_exit_status(0, readiness) == startup.EXIT_RENDERER_FAILED


def test_a_ready_renderer_keeps_the_main_loop_status():
    readiness = startup.RendererReadiness()
    readiness.mark_ready()

    assert readiness.ready
    assert startup.renderer_exit_status(0, readiness) == 0
    assert startup.renderer_exit_status(3, readiness) == 3


def test_failure_wins_over_an_earlier_ready():
    readiness = startup.RendererReadiness()
    readiness.mark_ready()
    readiness.mark_failed("surface lost")

    assert not readiness.ready
    assert startup.renderer_exit_status(0, readiness) == startup.EXIT_RENDERER_FAILED


# ---- entry point ---------------------------------------------------------------------------


def test_without_a_session_gtk_is_never_imported_and_the_exit_is_not_clean(
    runtime: Path, monkeypatch: pytest.MonkeyPatch
):
    monkeypatch.setenv("XDG_RUNTIME_DIR", str(runtime))
    monkeypatch.setenv("WAYLAND_DISPLAY", "wayland-0")
    monkeypatch.setenv("PIB_DISPLAY_WAYLAND_WAIT_SECONDS", "3")
    monkeypatch.setenv("GDK_BACKEND", "wayland")
    clock = Clock()
    monkeypatch.setattr(startup.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(startup.time, "sleep", clock.sleep)
    # Importing the GTK module would raise here.
    import display

    monkeypatch.delattr(display, "display_wayland_gtk", raising=False)
    monkeypatch.setitem(sys.modules, "display.display_wayland_gtk", None)

    status = startup.main()

    assert status == startup.EXIT_WAYLAND_UNAVAILABLE
    assert status != 0


def test_with_a_session_the_renderer_runs_and_its_status_is_returned(
    runtime: Path, monkeypatch: pytest.MonkeyPatch
):
    server = _listener(runtime / "wayland-0")
    monkeypatch.setenv("XDG_RUNTIME_DIR", str(runtime))
    monkeypatch.setenv("WAYLAND_DISPLAY", "wayland-0")
    # setenv first so that teardown restores the variable main() sets.
    monkeypatch.setenv("GDK_BACKEND", "unset-by-test")
    monkeypatch.delenv("GDK_BACKEND")
    seen = {}

    def run_renderer() -> int:
        seen["backend"] = os.environ.get("GDK_BACKEND")
        return 7

    renderer = types.ModuleType("display.display_wayland_gtk")
    renderer.run_renderer = run_renderer
    monkeypatch.setitem(sys.modules, "display.display_wayland_gtk", renderer)
    import display

    monkeypatch.setattr(display, "display_wayland_gtk", renderer, raising=False)
    try:
        status = startup.main()
    finally:
        server.close()

    assert status == 7
    assert seen["backend"] == "wayland"


def test_the_console_script_starts_through_the_gate():
    setup_py = (DISPLAY_ROOT / "setup.py").read_text(encoding="utf-8")
    assert '"display = display.display_startup:main"' in setup_py
    assert "display.display_wayland_gtk:main" not in setup_py
