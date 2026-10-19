"""Ready ordering and failure status of the GTK renderer and its launch file (PR-1980).

The unit environment has neither GTK, rclpy nor launch, so the module is imported with
stand-ins for gi, rclpy, PIL, std_msgs, datatypes and launch. They only record calls;
these tests prove the branches of display_wayland_gtk.py and launch.py, not GTK itself.
"""

from __future__ import annotations

import importlib
import importlib.util
import sys
import types
from pathlib import Path
from queue import Queue

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
DISPLAY_ROOT = REPO_ROOT / "ros_packages" / "display"
LAUNCH_FILE = DISPLAY_ROOT / "launch" / "launch.py"
sys.path.insert(0, str(DISPLAY_ROOT))

from display.display_startup import (  # noqa: E402
    EXIT_RENDERER_FAILED,
    RendererReadiness,
)
from display.display_web_request import SURFACE_READY, SURFACE_WEB  # noqa: E402

# ---- stand-ins -----------------------------------------------------------------------------


class Gui:
    """What the fake GTK does; each test sets the parts it needs."""

    def __init__(self):
        self.init_error: Exception | None = None
        self.default_display: object | None = object()
        self.realizes = True
        self.surface: object | None = object()
        self.monitors = Monitors(1)
        self.run_status = 0
        self.timeouts: list = []


class Monitors:
    def __init__(self, count: int):
        self.count = count
        self.handlers: list = []

    def get_n_items(self) -> int:
        return self.count

    def connect(self, signal: str, handler) -> int:
        assert signal == "items-changed"
        self.handlers.append(handler)
        return len(self.handlers)

    def add(self) -> None:
        self.count += 1
        for handler in list(self.handlers):
            handler(self, self.count - 1, 0, 1)


GUI = Gui()


class Widget:
    def __init__(self, *args, **kwargs):
        self.visible = False

    def __getattr__(self, name):
        return lambda *args, **kwargs: None


class FakeDisplay:
    def get_monitors(self):
        return GUI.monitors


class ApplicationWindow(Widget):
    def __init__(self, application=None):
        if GUI.init_error is not None:
            raise GUI.init_error
        super().__init__()
        self.realized = False
        self.application = application

    def realize(self):
        self.realized = GUI.realizes

    def get_realized(self):
        return self.realized

    def get_surface(self):
        return GUI.surface if self.realized else None

    def get_display(self):
        return FakeDisplay()

    def present(self):
        self.visible = True

    def hide(self):
        self.visible = False

    def is_visible(self):
        return self.visible


class Application:
    def __init__(self, application_id=None):
        self.application_id = application_id
        self.quit_called = False

    def run(self, _argv):
        self.do_activate()
        return GUI.run_status

    def quit(self):
        self.quit_called = True


class Picture(Widget):
    def set_paintable(self, paintable):
        self.paintable = paintable


def _gtk_modules() -> dict[str, types.ModuleType]:
    gi = types.ModuleType("gi")
    gi.require_version = lambda *_args: None
    repository = types.ModuleType("gi.repository")

    gtk = types.ModuleType("gi.repository.Gtk")
    gtk.Application = Application
    gtk.ApplicationWindow = ApplicationWindow
    gtk.Picture = Picture
    gtk.EventControllerKey = Widget
    gtk.GestureClick = Widget

    gdk = types.ModuleType("gi.repository.Gdk")
    gdk.Display = types.SimpleNamespace(get_default=lambda: GUI.default_display)
    gdk.KEY_Escape = 0xFF1B
    gdk.Texture = object
    gdk.MemoryFormat = types.SimpleNamespace(R8G8B8A8=0)
    gdk.MemoryTexture = types.SimpleNamespace(new=lambda *args: ("texture", args))

    glib = types.ModuleType("gi.repository.GLib")
    glib.Error = type("GLibError", (Exception,), {})
    glib.timeout_add = lambda ms, callback: GUI.timeouts.append((ms, callback))
    glib.Bytes = types.SimpleNamespace(new=lambda data: data)

    repository.Gtk, repository.Gdk, repository.GLib = gtk, gdk, glib
    gi.repository = repository
    return {
        "gi": gi,
        "gi.repository": repository,
    }


class Publisher:
    def __init__(self, topic: str):
        self.topic = topic
        self.sent: list[str] = []

    def publish(self, msg) -> None:
        self.sent.append(msg.data)


class Timer:
    def __init__(self, period: float, callback):
        self.period = period
        self.callback = callback
        self.cancelled = False

    def cancel(self) -> None:
        self.cancelled = True


class Logger:
    def __init__(self):
        self.lines: list[str] = []

    def info(self, message: str) -> None:
        self.lines.append(message)

    error = warning = info


class Node:
    def __init__(self, name: str):
        self.name = name
        self.publishers: dict[str, Publisher] = {}
        self.timers: list[Timer] = []
        self.logger = Logger()

    def create_subscription(self, *_args, **_kwargs):
        return None

    def create_publisher(self, _type, topic, _qos):
        self.publishers[topic] = Publisher(topic)
        return self.publishers[topic]

    def create_timer(self, period, callback):
        self.timers.append(Timer(period, callback))
        return self.timers[-1]

    def get_logger(self):
        return self.logger


def _ros_modules() -> dict[str, types.ModuleType]:
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda *args, **kwargs: None
    rclpy.shutdown = lambda *args, **kwargs: None
    executors = types.ModuleType("rclpy.executors")
    executors.SingleThreadedExecutor = object
    node = types.ModuleType("rclpy.node")
    node.Node = Node
    qos = types.ModuleType("rclpy.qos")
    qos.QoSProfile = lambda **kwargs: kwargs
    qos.DurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL="transient_local")
    qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST="keep_last")
    qos.ReliabilityPolicy = types.SimpleNamespace(RELIABLE="reliable")
    rclpy.executors, rclpy.node, rclpy.qos = executors, node, qos

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")

    class String:
        def __init__(self):
            self.data = ""

    std_msgs_msg.String = String
    datatypes = types.ModuleType("datatypes")
    datatypes_msg = types.ModuleType("datatypes.msg")
    datatypes_msg.DisplayImage = object
    datatypes_msg.ImageFormat = types.SimpleNamespace(ANIMATED_GIF=1)
    datatypes_msg.ImageId = types.SimpleNamespace(NONE=0, CUSTOM=1, PIB_EYES_ANIMATED=2)

    pil = types.ModuleType("PIL")
    for name in ("Image", "ImageDraw", "ImageFont"):
        module = types.ModuleType(f"PIL.{name}")
        module.LANCZOS = 1
        setattr(pil, name, module)
    return {
        "rclpy": rclpy,
        "rclpy.executors": executors,
        "rclpy.node": node,
        "rclpy.qos": qos,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
        "datatypes": datatypes,
        "datatypes.msg": datatypes_msg,
        "PIL": pil,
        "PIL.Image": pil.Image,
        "PIL.ImageDraw": pil.ImageDraw,
        "PIL.ImageFont": pil.ImageFont,
    }


@pytest.fixture
def gtk(monkeypatch: pytest.MonkeyPatch):
    global GUI
    GUI = Gui()
    for name, module in {**_gtk_modules(), **_ros_modules()}.items():
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.delitem(sys.modules, "display.display_wayland_gtk", raising=False)
    import display

    monkeypatch.delattr(display, "display_wayland_gtk", raising=False)
    module = importlib.import_module("display.display_wayland_gtk")
    monkeypatch.setattr(module, "DISPLAY_ON_DEMAND", True)
    monkeypatch.setenv("PIB_DISPLAY_PASSWORD_PROMPT", "0")
    yield module
    monkeypatch.delitem(sys.modules, "display.display_wayland_gtk", raising=False)


def _ready_messages(node) -> list[str]:
    return node.publishers["/pib/display_ready"].sent


def _tick(node) -> None:
    for timer in list(node.timers):
        if not timer.cancelled:
            timer.callback()


# ---- ready ordering ------------------------------------------------------------------------


def test_the_node_does_not_announce_ready_on_construction(gtk):
    node = gtk.DisplayNode(Queue(), RendererReadiness())

    assert _ready_messages(node) == []
    _tick(node)
    _tick(node)
    assert _ready_messages(node) == []


def test_ready_follows_a_usable_renderer_exactly_once(gtk):
    readiness = RendererReadiness()
    node = gtk.DisplayNode(Queue(), readiness)
    _tick(node)

    readiness.mark_ready()
    _tick(node)
    _tick(node)

    assert _ready_messages(node) == [SURFACE_READY]


def test_no_later_ready_is_sent_while_the_renderer_is_not_usable(gtk):
    node = gtk.DisplayNode(Queue(), RendererReadiness())

    node.publish_surface(SURFACE_WEB)
    node.publish_surface(SURFACE_READY)

    assert _ready_messages(node) == [SURFACE_WEB]


def test_a_failed_renderer_never_announces_ready(gtk):
    readiness = RendererReadiness()
    node = gtk.DisplayNode(Queue(), readiness)

    readiness.mark_failed("Gtk couldn't be initialized")
    _tick(node)
    node.publish_surface(SURFACE_READY)

    assert _ready_messages(node) == []


def test_the_password_prompt_waits_for_the_renderer(gtk, monkeypatch):
    monkeypatch.setenv("PIB_DISPLAY_PASSWORD_PROMPT", "1")
    decisions = []
    monkeypatch.setattr(
        gtk, "prompt_decision", lambda mode: decisions.append(mode) or "skip"
    )
    monkeypatch.setattr(gtk, "read_operating_mode", lambda: "normal")
    readiness = RendererReadiness()
    node = gtk.DisplayNode(Queue(), readiness)

    node.offer_password_prompt()
    assert decisions == []

    readiness.mark_ready()
    node.offer_password_prompt()
    assert decisions == []

    node.announce_ready()
    node.offer_password_prompt()
    assert decisions == ["normal"]
    assert _ready_messages(node) == [SURFACE_READY]


# ---- GTK window ----------------------------------------------------------------------------


def test_a_realized_window_on_an_output_makes_the_renderer_ready(gtk):
    readiness = RendererReadiness()
    app = gtk.DisplayApp(Queue(), readiness)

    app.do_activate()

    assert readiness.ready
    assert readiness.failure is None
    assert app.window.get_realized()
    # On demand: nothing is shown until a face or a text arrives.
    assert not app.window.is_visible()
    assert GUI.timeouts and GUI.timeouts[0][0] == 10


def test_gtk_initialisation_failure_is_recorded_and_ends_the_main_loop(gtk):
    GUI.init_error = RuntimeError(
        "Gtk couldn't be initialized. Use Gtk.init_check() if you want to handle this case."
    )
    readiness = RendererReadiness()
    app = gtk.DisplayApp(Queue(), readiness)

    app.do_activate()

    assert not readiness.ready
    assert "couldn't be initialized" in readiness.failure
    assert app.quit_called


def test_a_window_without_a_surface_is_a_failure(gtk):
    GUI.realizes = False
    readiness = RendererReadiness()
    app = gtk.DisplayApp(Queue(), readiness)

    app.do_activate()

    assert not readiness.ready
    assert "surface" in readiness.failure
    assert app.quit_called


def test_a_compositor_without_outputs_is_not_ready_until_one_appears(gtk):
    GUI.monitors = Monitors(0)
    readiness = RendererReadiness()
    app = gtk.DisplayApp(Queue(), readiness)

    app.do_activate()
    assert not readiness.ready
    assert readiness.failure is None
    assert not app.quit_called

    GUI.monitors.add()
    assert readiness.ready


def test_on_demand_show_and_hide_still_work_after_ready(gtk, monkeypatch):
    monkeypatch.setattr(gtk, "find_expression_path", lambda _expr: Path("happy.png"))
    monkeypatch.setattr(gtk, "load_rgba_from_path", lambda _path: b"rgba")
    app = gtk.DisplayApp(Queue(), RendererReadiness())
    app.do_activate()

    app._handle_command(gtk.DisplayCommand("expression", "happy"))
    assert app.window.is_visible()
    assert app.picture.paintable[0] == "texture"

    app._handle_command(gtk.DisplayCommand("hide"))
    assert not app.window.is_visible()

    app._handle_command(gtk.DisplayCommand("web_open"))
    app._handle_command(gtk.DisplayCommand("expression", "happy"))
    assert not app.window.is_visible()
    app._handle_command(gtk.DisplayCommand("web_hide"))
    assert not app.window.is_visible()


# ---- run_renderer --------------------------------------------------------------------------


class Threads:
    def __init__(self):
        self.started: list[tuple] = []

    def __call__(self, target, args=(), daemon=None):
        threads = self

        class Started:
            def start(self):
                threads.started.append((target, args, daemon))

        return Started()


def test_without_a_gtk_display_nothing_starts_and_the_status_is_a_failure(
    gtk, monkeypatch
):
    GUI.default_display = None
    threads = Threads()
    monkeypatch.setattr(gtk, "Thread", threads)

    assert gtk.run_renderer() == EXIT_RENDERER_FAILED
    assert threads.started == []


def test_a_working_renderer_starts_ros_and_returns_the_main_loop_status(
    gtk, monkeypatch
):
    threads = Threads()
    monkeypatch.setattr(gtk, "Thread", threads)

    assert gtk.run_renderer() == 0
    assert len(threads.started) == 1
    target, args, daemon = threads.started[0]
    assert target is gtk.run_ros
    assert daemon is True
    assert isinstance(args[1], RendererReadiness) and args[1].ready


def test_a_renderer_failure_after_gtk_init_is_not_a_clean_exit(gtk, monkeypatch):
    GUI.init_error = RuntimeError("Gtk couldn't be initialized.")
    monkeypatch.setattr(gtk, "Thread", Threads())

    assert gtk.run_renderer() == EXIT_RENDERER_FAILED


# ---- launch file ---------------------------------------------------------------------------


class LaunchNode:
    def __init__(self, **kwargs):
        self.kwargs = kwargs


class OnProcessExit:
    def __init__(self, *, target_action, on_exit):
        self.target_action = target_action
        self.on_exit = on_exit


class RegisterEventHandler:
    def __init__(self, event_handler):
        self.event_handler = event_handler


class LaunchDescription:
    def __init__(self, entities):
        self.entities = list(entities)


@pytest.fixture
def launch_description(monkeypatch: pytest.MonkeyPatch):
    launch = types.ModuleType("launch")
    launch.LaunchDescription = LaunchDescription
    actions = types.ModuleType("launch.actions")
    actions.RegisterEventHandler = RegisterEventHandler
    event_handlers = types.ModuleType("launch.event_handlers")
    event_handlers.OnProcessExit = OnProcessExit
    launch_ros = types.ModuleType("launch_ros")
    launch_ros_actions = types.ModuleType("launch_ros.actions")
    launch_ros_actions.Node = LaunchNode
    for name, module in {
        "launch": launch,
        "launch.actions": actions,
        "launch.event_handlers": event_handlers,
        "launch_ros": launch_ros,
        "launch_ros.actions": launch_ros_actions,
    }.items():
        monkeypatch.setitem(sys.modules, name, module)
    spec = importlib.util.spec_from_file_location("display_launch_file", LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def _exit_handler(description) -> OnProcessExit:
    handlers = [
        entity.event_handler
        for entity in description.entities
        if isinstance(entity, RegisterEventHandler)
    ]
    assert len(handlers) == 1
    return handlers[0]


def test_launch_starts_the_display_executable(launch_description):
    nodes = [e for e in launch_description.entities if isinstance(e, LaunchNode)]
    assert [n.kwargs for n in nodes] == [
        {"package": "display", "executable": "display"}
    ]
    assert _exit_handler(launch_description).target_action is nodes[0]


def test_launch_turns_a_failed_display_into_a_failed_launch(launch_description):
    handler = _exit_handler(launch_description)
    running = types.SimpleNamespace(is_shutdown=False)

    with pytest.raises(RuntimeError, match="exit status 75"):
        handler.on_exit(types.SimpleNamespace(returncode=75), running)
    with pytest.raises(RuntimeError, match="exit status 1"):
        handler.on_exit(types.SimpleNamespace(returncode=1), running)


def test_launch_accepts_a_clean_exit_and_a_stop_request(launch_description):
    handler = _exit_handler(launch_description)

    assert (
        handler.on_exit(
            types.SimpleNamespace(returncode=0),
            types.SimpleNamespace(is_shutdown=False),
        )
        is None
    )
    assert (
        handler.on_exit(
            types.SimpleNamespace(returncode=-2),
            types.SimpleNamespace(is_shutdown=True),
        )
        is None
    )
