from __future__ import annotations

import os
import re
import time
from dataclasses import dataclass
from hashlib import sha1
from io import BytesIO
from pathlib import Path
from queue import Empty, Queue
from threading import Thread
from typing import Optional

import PIL.Image
import rclpy
from PIL import Image as PILImage, ImageDraw, ImageFont
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from datatypes.msg import DisplayImage, ImageFormat, ImageId

import gi

gi.require_version("Gtk", "4.0")
gi.require_version("Gdk", "4.0")
gi.require_version("GLib", "2.0")

from gi.repository import Gdk, GLib, Gtk  # noqa: E402


STATIC_IMAGE_DIR = Path(os.getenv("STATIC_IMAGE_DIR", "/app/ros2_ws/display/static_images"))
EXPRESSION_DIR = Path(os.getenv("PIB_EXPRESSION_DIR", "/app/pib-expression-faces"))

DISPLAY_WIDTH = int(os.getenv("PIB_DISPLAY_WIDTH", "1024"))
DISPLAY_HEIGHT = int(os.getenv("PIB_DISPLAY_HEIGHT", "600"))
DISPLAY_TEXT_MAX_CHARS = int(os.getenv("PIB_DISPLAY_TEXT_MAX_CHARS", "40"))
DISPLAY_TEXT_PADDING = int(os.getenv("PIB_DISPLAY_TEXT_PADDING", "40"))
DISPLAY_ON_DEMAND = os.getenv("PIB_DISPLAY_ON_DEMAND", "1") == "1"


_ros_logger: Optional[Node] = None


def set_ros_logger(node: Node) -> None:
    global _ros_logger
    _ros_logger = node


def log_info(message: str) -> None:
    if _ros_logger is not None:
        _ros_logger.get_logger().info(message)
    else:
        print(f"[display-v2] {message}", flush=True)


@dataclass(frozen=True)
class RawImage:
    format_value: int
    data: bytes


@dataclass(frozen=True)
class DisplayCommand:
    kind: str  # "expression" | "text" | "raw" | "hide"
    value: object = None


def normalize_expression(value: str) -> str:
    value = value.strip().lower()
    value = value.replace("-", "_").replace(" ", "_")
    return re.sub(r"[^a-z0-9_]", "", value)


def memory_texture_from_rgba(rgba: bytes, width: int, height: int) -> Gdk.Texture:
    stride = width * 4
    glib_bytes = GLib.Bytes.new(rgba)
    return Gdk.MemoryTexture.new(width, height, Gdk.MemoryFormat.R8G8B8A8, glib_bytes, stride)


class TextRenderer:
    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height

    def render_rgba(self, value: str) -> bytes:
        value = str(value).strip()[:DISPLAY_TEXT_MAX_CHARS]
        image = PILImage.new("RGBA", (self.width, self.height), (0, 0, 0, 255))
        draw = ImageDraw.Draw(image)
        color = (69, 183, 255, 255)

        best_font = self.load_font(20)
        best_lines = [value]
        max_width = self.width - 2 * DISPLAY_TEXT_PADDING
        max_height = self.height - 2 * DISPLAY_TEXT_PADDING

        for size in range(180, 18, -4):
            font = self.load_font(size)
            lines = self.wrap_text(draw, value, font, max_width)
            boxes = [draw.textbbox((0, 0), line, font=font) for line in lines]
            heights = [box[3] - box[1] for box in boxes]
            total_height = sum(heights) + max(0, len(lines) - 1) * int(size * 0.25)
            max_line_width = max((box[2] - box[0]) for box in boxes) if boxes else 0
            if max_line_width <= max_width and total_height <= max_height:
                best_font = font
                best_lines = lines
                break

        boxes = [draw.textbbox((0, 0), line, font=best_font) for line in best_lines]
        heights = [box[3] - box[1] for box in boxes]
        spacing = 16
        total_height = sum(heights) + max(0, len(best_lines) - 1) * spacing
        y = (self.height - total_height) // 2

        for line, box, height in zip(best_lines, boxes, heights):
            line_width = box[2] - box[0]
            x = (self.width - line_width) // 2
            draw.text((x, y), line, font=best_font, fill=color)
            y += height + spacing

        return image.tobytes()

    @staticmethod
    def load_font(size: int):
        candidates = [
            "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        ]
        for candidate in candidates:
            try:
                return ImageFont.truetype(candidate, size)
            except Exception:
                pass
        return ImageFont.load_default()

    @staticmethod
    def wrap_text(draw, value: str, font, max_width: int) -> list[str]:
        words = value.split()
        if not words:
            return [""]
        lines = []
        line = words[0]
        for word in words[1:]:
            test = line + " " + word
            box = draw.textbbox((0, 0), test, font=font)
            if box[2] - box[0] <= max_width:
                line = test
            else:
                lines.append(line)
                line = word
        lines.append(line)
        return lines


class DisplayApp(Gtk.Application):
    def __init__(self, command_queue: Queue[DisplayCommand]):
        super().__init__(application_id="pib.display.wayland")
        self.command_queue = command_queue
        self.window: Gtk.ApplicationWindow | None = None
        self.picture: Gtk.Picture | None = None
        self.text_renderer = TextRenderer(DISPLAY_WIDTH, DISPLAY_HEIGHT)

    def do_activate(self) -> None:  # type: ignore[override]
        if self.window is None:
            self.window = Gtk.ApplicationWindow(application=self)
            self.window.set_default_size(DISPLAY_WIDTH, DISPLAY_HEIGHT)
            self.window.fullscreen()
            self.window.set_decorated(False)

            self.picture = Gtk.Picture()
            self.picture.set_can_shrink(False)
            self.picture.set_keep_aspect_ratio(False)
            self.window.set_child(self.picture)

            controller = Gtk.EventControllerKey()
            controller.connect("key-pressed", self._on_key)
            self.window.add_controller(controller)

            click = Gtk.GestureClick()
            click.connect("pressed", self._on_click)
            self.window.add_controller(click)

            if DISPLAY_ON_DEMAND:
                self.window.hide()
            else:
                self.window.present()

            GLib.timeout_add(10, self._poll_commands)

        if not DISPLAY_ON_DEMAND:
            self.window.present()

    def _ensure_visible(self) -> None:
        if self.window is None:
            return
        if not self.window.is_visible():
            self.window.present()

    def _hide(self) -> None:
        if self.window is None:
            return
        self.window.hide()
        log_info("display dismissed by user")

    def _on_key(self, _controller, keyval, _keycode, _state) -> bool:
        if keyval == Gdk.KEY_Escape:
            self._hide()
            return True
        return False

    def _on_click(self, _gesture, _n_press, _x, _y) -> None:
        self._hide()

    def _poll_commands(self) -> bool:
        try:
            while True:
                cmd = self.command_queue.get_nowait()
                self._handle_command(cmd)
        except Empty:
            pass
        return True

    def _handle_command(self, cmd: DisplayCommand) -> None:
        if self.window is None or self.picture is None:
            return

        if cmd.kind == "hide":
            self._hide()
            return

        if cmd.kind == "text":
            self._ensure_visible()
            rgba = self.text_renderer.render_rgba(str(cmd.value))
            tex = memory_texture_from_rgba(rgba, DISPLAY_WIDTH, DISPLAY_HEIGHT)
            self.picture.set_paintable(tex)
            return

        if cmd.kind == "expression":
            self._ensure_visible()
            expr = normalize_expression(str(cmd.value))
            path = find_expression_path(expr)
            if path is None:
                log_info(f"expression not found: {expr}")
                return
            rgba = load_rgba_from_path(path)
            tex = memory_texture_from_rgba(rgba, DISPLAY_WIDTH, DISPLAY_HEIGHT)
            self.picture.set_paintable(tex)
            log_info(f"expression shown: {expr}")
            return

        if cmd.kind == "raw":
            self._ensure_visible()
            rgba = load_rgba_from_raw(cmd.value)
            tex = memory_texture_from_rgba(rgba, DISPLAY_WIDTH, DISPLAY_HEIGHT)
            self.picture.set_paintable(tex)


def find_expression_path(expression: str) -> Path | None:
    if not EXPRESSION_DIR.exists():
        return None
    normalized = normalize_expression(expression)
    for path in sorted(EXPRESSION_DIR.iterdir()):
        if not path.is_file() or path.suffix.lower() not in (".png", ".gif", ".jpg", ".jpeg"):
            continue
        if normalize_expression(path.stem) == normalized:
            return path
    return None


def load_rgba_from_path(path: Path) -> bytes:
    with PIL.Image.open(path) as image:
        frame = image.convert("RGBA")
        if frame.size != (DISPLAY_WIDTH, DISPLAY_HEIGHT):
            frame = frame.resize((DISPLAY_WIDTH, DISPLAY_HEIGHT), PIL.Image.LANCZOS)
        return frame.tobytes()


def load_rgba_from_raw(raw: RawImage) -> bytes:
    with PIL.Image.open(BytesIO(raw.data)) as image:
        frame = image.convert("RGBA")
        if frame.size != (DISPLAY_WIDTH, DISPLAY_HEIGHT):
            frame = frame.resize((DISPLAY_WIDTH, DISPLAY_HEIGHT), PIL.Image.LANCZOS)
        return frame.tobytes()


class DisplayNode(Node):
    def __init__(self, command_queue: Queue[DisplayCommand]) -> None:
        super().__init__("display")
        set_ros_logger(self)
        self.command_queue = command_queue

        display_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(DisplayImage, "/display_image", self.on_display_image, display_qos)
        self.create_subscription(String, "/pib/expression", self.on_expression, 10)
        self.create_subscription(String, "/pib/display_text", self.on_text, 10)
        self.create_subscription(String, "/pib/display_hide", self.on_hide, 10)
        self.ready_pub = self.create_publisher(String, "/pib/display_ready", display_qos)

        msg = String()
        msg.data = "ready"
        self.ready_pub.publish(msg)

        self.get_logger().info("Display V2 running (Wayland/GTK4)")
        self.get_logger().info("published /pib/display_ready")

    def on_expression(self, msg: String) -> None:
        self.get_logger().info(f"expression requested: {msg.data}")
        self.command_queue.put(DisplayCommand("expression", msg.data))

    def on_text(self, msg: String) -> None:
        self.get_logger().info("display text requested")
        self.command_queue.put(DisplayCommand("text", msg.data))

    def on_hide(self, msg: String) -> None:
        self.command_queue.put(DisplayCommand("hide"))

    def on_display_image(self, msg: DisplayImage) -> None:
        raw = self.raw_from_display_image(msg)
        if raw is not None:
            self.command_queue.put(DisplayCommand("raw", raw))

    @staticmethod
    def raw_from_display_image(msg: DisplayImage) -> RawImage | None:
        if msg.id.value == ImageId.NONE:
            return None
        if msg.id.value == ImageId.CUSTOM:
            return RawImage(msg.format.value, b"".join(msg.data))
        if msg.id.value == ImageId.PIB_EYES_ANIMATED:
            path = STATIC_IMAGE_DIR / "pib-eyes-animated.gif"
            return RawImage(ImageFormat.ANIMATED_GIF, path.read_bytes())
        return None


def run_ros(command_queue: Queue[DisplayCommand]) -> None:
    rclpy.init()
    executor = SingleThreadedExecutor()
    node = DisplayNode(command_queue)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


def main(args=None) -> None:
    # Force Wayland backend; this prevents accidental X fallback.
    os.environ.setdefault("GDK_BACKEND", "wayland")

    command_queue: Queue[DisplayCommand] = Queue(maxsize=200)
    Thread(target=run_ros, args=(command_queue,), daemon=True).start()

    app = DisplayApp(command_queue)
    app.run(None)


if __name__ == "__main__":
    main()

