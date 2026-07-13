from __future__ import annotations

import base64
from collections import OrderedDict
import os
import re
import time
from dataclasses import dataclass
from hashlib import sha1
from io import BytesIO
from itertools import cycle
from pathlib import Path
from queue import Empty, Queue
from threading import Event as ThreadEvent, Lock, Thread
from typing import Iterable, Iterator, Optional

import PIL.Image
import rclpy
from PIL import Image as PILImage, ImageDraw, ImageFont
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tkinter import *

from datatypes.msg import DisplayImage, ImageFormat, ImageId


os.environ.setdefault("DISPLAY", ":0")

STATIC_IMAGE_DIR = Path(os.getenv("STATIC_IMAGE_DIR", "/app/ros2_ws/display/static_images"))
EXPRESSION_DIR = Path(os.getenv("PIB_EXPRESSION_DIR", "/app/pib-expression-faces"))

DISPLAY_WIDTH = int(os.getenv("PIB_DISPLAY_WIDTH", "0"))
DISPLAY_HEIGHT = int(os.getenv("PIB_DISPLAY_HEIGHT", "0"))
DISPLAY_TEXT_MAX_CHARS = int(os.getenv("PIB_DISPLAY_TEXT_MAX_CHARS", "40"))
DISPLAY_TEXT_PADDING = int(os.getenv("PIB_DISPLAY_TEXT_PADDING", "40"))
DISPLAY_POLL_MS = int(os.getenv("PIB_DISPLAY_POLL_MS", "5"))
DISPLAY_IDLE_SECONDS = float(os.getenv("PIB_DISPLAY_IDLE_SECONDS", "10"))
DISPLAY_ON_DEMAND = os.getenv("PIB_DISPLAY_ON_DEMAND", "1") == "1"
DISPLAY_VERBOSE = os.getenv("PIB_DISPLAY_NODE_VERBOSE", "0") == "1"
DISPLAY_TEXT_CACHE_MAX = int(os.getenv("PIB_DISPLAY_TEXT_CACHE_MAX", "50"))
PRELOAD_POLL_MS = int(os.getenv("PIB_DISPLAY_PRELOAD_POLL_MS", "10"))

TEXT_CANVAS_WIDTH = int(os.getenv("PIB_DISPLAY_TEXT_WIDTH", "800"))
TEXT_CANVAS_HEIGHT = int(os.getenv("PIB_DISPLAY_TEXT_HEIGHT", "480"))

_ros_logger: Optional[Node] = None


def set_ros_logger(node: Node) -> None:
    global _ros_logger
    _ros_logger = node


def log_debug(message: str) -> None:
    if not DISPLAY_VERBOSE:
        return
    if _ros_logger is not None:
        _ros_logger.get_logger().info(f"[display-v2-debug] {message}")
    else:
        print(f"[display-v2-debug] {time.monotonic():.6f} {message}", flush=True)


def log_info(message: str) -> None:
    if _ros_logger is not None:
        _ros_logger.get_logger().info(message)
    else:
        print(f"[display-v2] {message}", flush=True)


@dataclass(frozen=True)
class RawImage:
    format_value: int
    data: bytes


@dataclass
class AnimationFrame:
    duration_ms: int
    photo_image: PhotoImage


@dataclass(frozen=True)
class PreloadItem:
    cache_key: str
    kind: str
    frames: list[tuple[int, bytes]]


class Animation:
    def __init__(self, frames: Iterable[AnimationFrame]):
        self.frames = list(frames)
        self.iterator = iter(cycle(self.frames))
        self.stopped = False

    def stop(self) -> None:
        self.stopped = True

    def __iter__(self) -> Iterator[AnimationFrame]:
        return self

    def __next__(self) -> AnimationFrame:
        if self.stopped:
            raise StopIteration()
        return next(self.iterator)


@dataclass(frozen=True)
class DisplayCommand:
    kind: str
    value: object = None


class GuiLifecycle:
    def __init__(
        self,
        command_queue: Queue[DisplayCommand],
        preload_queue: Queue[PreloadItem | None],
    ):
        self.command_queue = command_queue
        self.preload_queue = preload_queue
        self._thread: Thread | None = None
        self._lock = Lock()
        self._tk_ready = ThreadEvent()

    def ensure_started(self) -> None:
        if not DISPLAY_ON_DEMAND:
            return

        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return

            self._tk_ready.clear()
            self._thread = Thread(
                target=run_gui,
                args=(self.command_queue, self.preload_queue, self),
                daemon=True,
            )
            self._thread.start()

        if not self._tk_ready.wait(timeout=10.0):
            log_info("warning: display GUI did not become ready within 10s")

    def mark_tk_ready(self) -> None:
        self._tk_ready.set()


class ImageCache:
    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height
        self.static_images: dict[str, PhotoImage] = {}
        self.animations: dict[str, list[AnimationFrame]] = {}

    def apply_preload_item(self, item: PreloadItem) -> None:
        if item.kind == "static":
            self.static_images[item.cache_key] = self._photo_from_png_bytes(item.frames[0][1])
            return

        frames = [
            AnimationFrame(duration_ms, self._photo_from_png_bytes(png_bytes))
            for duration_ms, png_bytes in item.frames
        ]
        self.animations[item.cache_key] = frames

    def lazy_load_expression(self, expression: str) -> tuple[str, object] | None:
        path = self.find_expression_path(expression)
        if path is None:
            return None

        normalized = self.normalize_expression(expression)
        key = f"expression:{normalized}"
        raw = RawImage(self.format_from_path(path), path.read_bytes())

        if raw.format_value == ImageFormat.ANIMATED_GIF:
            self.animations[key] = self._build_animation_frames(raw)
            return ("animation", self.animations[key])

        self.static_images[key] = self._build_static_photo(raw)
        return ("static", self.static_images[key])

    def find_expression_path(self, expression: str) -> Path | None:
        if not EXPRESSION_DIR.exists():
            return None

        normalized = self.normalize_expression(expression)
        for path in sorted(EXPRESSION_DIR.iterdir()):
            if not path.is_file() or path.suffix.lower() not in (".png", ".gif", ".jpg", ".jpeg"):
                continue
            if self.normalize_expression(path.stem) == normalized:
                return path
        return None

    def get_expression(self, expression: str) -> tuple[str, object] | None:
        expression = self.normalize_expression(expression)
        key = f"expression:{expression}"

        if key in self.static_images:
            return ("static", self.static_images[key])

        if key in self.animations:
            return ("animation", self.animations[key])

        return None

    def get_default_animation(self) -> list[AnimationFrame] | None:
        return self.animations.get("default")

    def get_raw_image(self, raw: RawImage) -> tuple[str, object]:
        key = f"raw:{raw.format_value}:{sha1(raw.data).hexdigest()}"

        if raw.format_value == ImageFormat.ANIMATED_GIF:
            if key not in self.animations:
                self.animations[key] = self._build_animation_frames(raw)
            return ("animation", self.animations[key])

        if key not in self.static_images:
            self.static_images[key] = self._build_static_photo(raw)
        return ("static", self.static_images[key])

    @staticmethod
    def normalize_expression(value: str) -> str:
        value = value.strip().lower()
        value = value.replace("-", "_").replace(" ", "_")
        return re.sub(r"[^a-z0-9_]", "", value)

    @staticmethod
    def format_from_path(path: Path) -> int:
        suffix = path.suffix.lower()
        if suffix == ".gif":
            return ImageFormat.ANIMATED_GIF
        if suffix == ".png":
            return ImageFormat.PNG
        if suffix in (".jpg", ".jpeg"):
            return ImageFormat.JPEG
        raise RuntimeError(f"unsupported image format: {path}")

    @staticmethod
    def format_to_str(format_value: int) -> str:
        if format_value == ImageFormat.ANIMATED_GIF:
            return "gif"
        if format_value == ImageFormat.PNG:
            return "png"
        if format_value == ImageFormat.JPEG:
            return "jpeg"
        raise RuntimeError(f"unsupported image format value: {format_value}")

    @staticmethod
    def _photo_from_png_bytes(png_bytes: bytes) -> PhotoImage:
        data = base64.b64encode(png_bytes)
        return PhotoImage(data=data, format="png")

    def _build_static_photo(self, raw: RawImage) -> PhotoImage:
        start = time.monotonic()
        format_str = self.format_to_str(raw.format_value)

        with PIL.Image.open(BytesIO(raw.data)) as image:
            if image.size == (self.width, self.height):
                resized = image
            else:
                resized = image.resize((self.width, self.height))

            buffer = BytesIO()
            resized.save(buffer, format_str)
            data = base64.b64encode(buffer.getvalue())

        photo = PhotoImage(data=data, format=format_str)
        log_debug(f"static photo built in {(time.monotonic() - start) * 1000:.1f} ms")
        return photo

    def _build_animation_frames(self, raw: RawImage) -> list[AnimationFrame]:
        start = time.monotonic()
        frames: list[AnimationFrame] = []

        with PIL.Image.open(BytesIO(raw.data)) as image:
            for index in range(image.n_frames):
                image.seek(index)
                if image.size == (self.width, self.height):
                    resized = image.copy()
                else:
                    resized = image.resize((self.width, self.height))

                buffer = BytesIO()
                resized.save(buffer, "gif")
                data = base64.b64encode(buffer.getvalue())
                duration_ms = image.info.get("duration", 80)

                frames.append(AnimationFrame(duration_ms, PhotoImage(data=data)))

        log_debug(f"animation built frames={len(frames)} in {(time.monotonic() - start) * 1000:.1f} ms")
        return frames


class AsyncImagePreloader:
    def __init__(
        self,
        width: int,
        height: int,
        result_queue: Queue[PreloadItem | None],
    ):
        self.width = width
        self.height = height
        self.result_queue = result_queue
        self._thread: Thread | None = None

    def start(self) -> None:
        self._thread = Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        try:
            self._preload_default_eyes()
            self._preload_expressions()
        finally:
            self.result_queue.put(None)

    def _preload_default_eyes(self) -> None:
        path = STATIC_IMAGE_DIR / "pib-eyes-animated.gif"
        if not path.exists():
            raise FileNotFoundError(f"default eyes missing: {path}")

        item = self._build_preload_item("default", "animation", path)
        self.result_queue.put(item)
        log_info("default animated eyes preloaded (background)")

    def _preload_expressions(self) -> None:
        if not EXPRESSION_DIR.exists():
            log_info(f"expression directory missing: {EXPRESSION_DIR}")
            return

        start = time.monotonic()
        count = 0

        for path in sorted(EXPRESSION_DIR.iterdir()):
            if not path.is_file() or path.suffix.lower() not in (".png", ".gif", ".jpg", ".jpeg"):
                continue

            expression = ImageCache.normalize_expression(path.stem)
            key = f"expression:{expression}"
            kind = "animation" if path.suffix.lower() == ".gif" else "static"
            self.result_queue.put(self._build_preload_item(key, kind, path))
            count += 1

        log_info(f"preloaded {count} expression files in {(time.monotonic() - start) * 1000:.1f} ms (background)")

    def _build_preload_item(self, cache_key: str, kind: str, path: Path) -> PreloadItem:
        if kind == "static":
            frames = [(0, self._encode_static_frame(path))]
            return PreloadItem(cache_key, kind, frames)

        return PreloadItem(cache_key, kind, self._encode_animation_frames(path))

    def _encode_static_frame(self, path: Path) -> bytes:
        with PIL.Image.open(path) as image:
            frame = image.copy()
            if frame.size != (self.width, self.height):
                frame = frame.resize((self.width, self.height), PIL.Image.LANCZOS)

            buffer = BytesIO()
            frame.convert("RGBA").save(buffer, format="PNG", optimize=True)
            return buffer.getvalue()

    def _encode_animation_frames(self, path: Path) -> list[tuple[int, bytes]]:
        frames: list[tuple[int, bytes]] = []

        with PIL.Image.open(path) as image:
            for index in range(image.n_frames):
                image.seek(index)
                frame = image.copy()
                if frame.size != (self.width, self.height):
                    frame = frame.resize((self.width, self.height), PIL.Image.LANCZOS)

                buffer = BytesIO()
                frame.convert("RGBA").save(buffer, format="PNG", optimize=True)
                duration_ms = int(image.info.get("duration", 80))
                frames.append((duration_ms, buffer.getvalue()))

        return frames


class TextRenderer:
    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height
        self.cache: OrderedDict[str, PhotoImage] = OrderedDict()

    def render(self, value: str) -> PhotoImage:
        value = str(value).strip()[:DISPLAY_TEXT_MAX_CHARS]

        if value in self.cache:
            photo = self.cache.pop(value)
            self.cache[value] = photo
            return photo

        start = time.monotonic()
        photo = self._build_text_photo(value)
        self.cache[value] = photo

        while len(self.cache) > DISPLAY_TEXT_CACHE_MAX:
            self.cache.popitem(last=False)

        log_debug(f"text rendered in {(time.monotonic() - start) * 1000:.1f} ms")
        return photo

    def _build_text_photo(self, value: str) -> PhotoImage:
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

        buffer = BytesIO()
        image.save(buffer, "png")
        data = base64.b64encode(buffer.getvalue())
        return PhotoImage(data=data, format="png")

    @staticmethod
    def load_font(size: int):
        candidates = [
            "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
            "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf",
            "/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf",
            "/usr/share/fonts/truetype/msttcorefonts/Arial.ttf",
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


class DisplayController:
    def __init__(self, canvas: Canvas, root: Widget, image_cache: ImageCache, text_renderer: TextRenderer):
        self.canvas = canvas
        self.root = root
        self.image_cache = image_cache
        self.text_renderer = text_renderer
        self.current_content: PhotoImage | Animation | None = None
        self.default_timer_id: Optional[str] = None
        self.animation_generation = 0
        self.window_visible = not DISPLAY_ON_DEMAND

    def reveal_window(self) -> None:
        if self.window_visible:
            return

        toplevel = self.root.winfo_toplevel()
        toplevel.deiconify()
        toplevel.lift()
        self.window_visible = True

    def hide_window(self) -> None:
        self.cancel_default_timer()
        self.stop_current_animation()
        self.canvas.delete("all")
        self.current_content = None
        self.canvas.configure(background="black")

        if DISPLAY_ON_DEMAND:
            self.root.winfo_toplevel().withdraw()
            self.window_visible = False
            log_info("display hidden until next trigger")

    def show_placeholder(self) -> None:
        self.stop_current_animation()
        self.canvas.delete("all")
        self.current_content = None
        self.canvas.configure(background="black")

    def show_default(self) -> None:
        self.reveal_window()
        self.cancel_default_timer()
        frames = self.image_cache.get_default_animation()
        if frames is None:
            self.show_placeholder()
            return
        self.show_animation(frames)
        self.schedule_idle_action()

    def show_expression(self, expression: str, preload_state: str = "ready") -> None:
        start = time.monotonic()
        result = self.image_cache.get_expression(expression)

        if result is None:
            if preload_state == "loading":
                log_info(f"expression not ready yet: {expression}")
                result = self.image_cache.lazy_load_expression(expression)

            if result is None:
                log_info(f"expression not found: {expression}")
                return

        self.reveal_window()
        kind, payload = result
        self.show_payload(kind, payload)
        self.schedule_idle_action()

        log_info(f"expression shown: {expression} total={(time.monotonic() - start) * 1000:.1f} ms")

    def show_text(self, value: str) -> None:
        start = time.monotonic()
        self.reveal_window()
        photo = self.text_renderer.render(value)
        self.show_photo(photo)
        self.schedule_idle_action()
        log_info(f"text shown: {str(value)[:DISPLAY_TEXT_MAX_CHARS]} total={(time.monotonic() - start) * 1000:.1f} ms")

    def show_raw(self, raw: RawImage) -> None:
        self.reveal_window()
        kind, payload = self.image_cache.get_raw_image(raw)
        self.show_payload(kind, payload)
        self.schedule_idle_action()

    def show_payload(self, kind: str, payload: object) -> None:
        if kind == "static":
            self.show_photo(payload)
        elif kind == "animation":
            self.show_animation(payload)
        else:
            log_info(f"unknown payload kind: {kind}")

    def show_photo(self, photo: PhotoImage) -> None:
        self.stop_current_animation()
        self.canvas.delete("all")
        self.current_content = photo
        self.canvas.create_image(0, 0, image=self.current_content, anchor="nw")

    def show_animation(self, frames: list[AnimationFrame]) -> None:
        self.stop_current_animation()
        self.canvas.delete("all")
        animation = Animation(frames)
        self.current_content = animation
        generation = self.animation_generation
        self.show_next_frame(animation, generation)

    def show_next_frame(self, animation: Animation, generation: int) -> None:
        if generation != self.animation_generation:
            return

        try:
            frame = next(animation)
        except StopIteration:
            return

        if generation != self.animation_generation:
            return

        self.canvas.delete("all")
        self.canvas.create_image(0, 0, image=frame.photo_image, anchor="nw")
        self.canvas.after(frame.duration_ms, self.show_next_frame, animation, generation)

    def stop_current_animation(self) -> None:
        self.animation_generation += 1
        if isinstance(self.current_content, Animation):
            self.current_content.stop()

    def schedule_idle_action(self) -> None:
        self.cancel_default_timer()
        if DISPLAY_IDLE_SECONDS <= 0:
            return

        if DISPLAY_ON_DEMAND:
            callback = self.hide_window
        else:
            callback = self.show_default

        self.default_timer_id = self.canvas.after(int(DISPLAY_IDLE_SECONDS * 1000), callback)

    def schedule_default(self) -> None:
        self.schedule_idle_action()

    def cancel_default_timer(self) -> None:
        if self.default_timer_id is not None:
            try:
                self.canvas.after_cancel(self.default_timer_id)
            except Exception:
                pass
            self.default_timer_id = None


class GuiApplication(Frame):
    def __init__(
        self,
        parent: Widget,
        command_queue: Queue[DisplayCommand],
        preload_queue: Queue[PreloadItem | None],
        gui_lifecycle: GuiLifecycle | None = None,
        *args,
        **kwargs,
    ):
        self.width = kwargs.setdefault("width", 100)
        self.height = kwargs.setdefault("height", 100)

        Frame.__init__(self, parent, *args, **kwargs)

        self.command_queue = command_queue
        self.preload_queue = preload_queue
        self.preload_state = "loading"
        self.default_ready = False
        self.preload_complete = False

        self.canvas = Canvas(
            self,
            width=self.width,
            height=self.height,
            borderwidth=0,
            highlightthickness=0,
            background="black",
        )
        self.canvas.place(x=0, y=0)

        self.image_cache = ImageCache(self.width, self.height)
        self.text_renderer = TextRenderer(self.width, self.height)
        self.controller = DisplayController(self.canvas, parent, self.image_cache, self.text_renderer)

        self.controller.show_placeholder()
        if not DISPLAY_ON_DEMAND:
            AsyncImagePreloader(self.width, self.height, self.preload_queue).start()
        self._poll_preload()
        self._poll_commands()
        self.grid()

        if gui_lifecycle is not None:
            gui_lifecycle.mark_tk_ready()

    def _poll_preload(self) -> None:
        try:
            while True:
                item = self.preload_queue.get_nowait()
                if item is None:
                    self.preload_complete = True
                    if self.preload_state != "ready":
                        self.preload_state = "ready"
                        log_info("preload complete")
                        if not DISPLAY_ON_DEMAND and self.default_ready:
                            self.controller.show_default()
                    break

                self.image_cache.apply_preload_item(item)
                if item.cache_key == "default" and not self.default_ready:
                    self.default_ready = True
                    self.preload_state = "ready"
                    log_info("default eyes preloaded")
                    if not DISPLAY_ON_DEMAND:
                        self.controller.show_default()
        except Empty:
            pass

        if not self.preload_complete:
            self.after(PRELOAD_POLL_MS, self._poll_preload)

    def _poll_commands(self) -> None:
        try:
            while True:
                command = self.command_queue.get_nowait()
                self.handle_command(command)
        except Empty:
            pass

        self.after(DISPLAY_POLL_MS, self._poll_commands)

    def handle_command(self, command: DisplayCommand) -> None:
        if command.kind == "expression":
            self.controller.show_expression(str(command.value), self.preload_state)
        elif command.kind == "text":
            self.controller.show_text(str(command.value))
        elif command.kind == "raw":
            self.controller.show_raw(command.value)
        elif command.kind == "default":
            self.controller.show_default()
        elif command.kind == "hide":
            self.controller.hide_window()
        elif command.kind == "quit":
            self.winfo_toplevel().destroy()


class DisplayNode(Node):
    def __init__(self, command_queue: Queue[DisplayCommand], gui_lifecycle: GuiLifecycle) -> None:
        super().__init__("display")
        set_ros_logger(self)
        self.command_queue = command_queue
        self.gui_lifecycle = gui_lifecycle

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

        ready_msg = String()
        ready_msg.data = "ready"
        self.ready_pub.publish(ready_msg)

        self.get_logger().info("Display V2 running")
        self.get_logger().info(f"Expression directory: {EXPRESSION_DIR}")
        if DISPLAY_ON_DEMAND:
            self.get_logger().info("On-demand mode: GUI opens only when triggered")
            self.get_logger().info("published /pib/display_ready")

    def enqueue(self, command: DisplayCommand) -> None:
        self.gui_lifecycle.ensure_started()
        self.command_queue.put(command)

    def on_expression(self, msg: String) -> None:
        self.get_logger().info(f"expression requested: {msg.data}")
        self.enqueue(DisplayCommand("expression", msg.data))

    def on_text(self, msg: String) -> None:
        self.get_logger().info(f"display text requested: {msg.data[:DISPLAY_TEXT_MAX_CHARS]}")
        self.enqueue(DisplayCommand("text", msg.data))

    def on_hide(self, msg: String) -> None:
        self.get_logger().info("display hide requested")
        self.enqueue(DisplayCommand("hide"))

    def on_display_image(self, msg: DisplayImage) -> None:
        try:
            raw = self.raw_from_display_image(msg)
            if raw is not None:
                self.enqueue(DisplayCommand("raw", raw))
        except Exception as exc:
            self.get_logger().error(f"DisplayImage conversion failed: {exc}")

    @staticmethod
    def raw_from_display_image(msg: DisplayImage) -> RawImage | None:
        if msg.id.value == ImageId.NONE:
            return None

        if msg.id.value == ImageId.CUSTOM:
            return RawImage(msg.format.value, b"".join(msg.data))

        if msg.id.value == ImageId.PIB_EYES_ANIMATED:
            path = STATIC_IMAGE_DIR / "pib-eyes-animated.gif"
            return RawImage(ImageFormat.ANIMATED_GIF, path.read_bytes())

        raise RuntimeError(f"unsupported image id: {msg.id.value}")


def resolve_display_size(root: Tk) -> tuple[int, int]:
    if DISPLAY_WIDTH > 0 and DISPLAY_HEIGHT > 0:
        return DISPLAY_WIDTH, DISPLAY_HEIGHT

    root.update_idletasks()
    return root.winfo_screenwidth(), root.winfo_screenheight()


def run_gui(
    command_queue: Queue[DisplayCommand],
    preload_queue: Queue[PreloadItem | None],
    gui_lifecycle: GuiLifecycle | None = None,
) -> None:
    root = Tk()

    def on_escape(_event) -> None:
        if DISPLAY_ON_DEMAND:
            command_queue.put(DisplayCommand("hide"))
        else:
            root.destroy()

    root.bind("<Escape>", on_escape)

    root.update_idletasks()
    width, height = resolve_display_size(root)
    root.geometry(f"{width}x{height}+0+0")
    root.overrideredirect(True)
    root.attributes("-fullscreen", True)

    if DISPLAY_ON_DEMAND:
        root.withdraw()
    else:
        root.lift()
        root.focus_force()

    GuiApplication(root, command_queue, preload_queue, gui_lifecycle, width=width, height=height)
    root.mainloop()


def run_ros(command_queue: Queue[DisplayCommand], gui_lifecycle: GuiLifecycle) -> None:
    rclpy.init()
    executor = SingleThreadedExecutor()
    node = DisplayNode(command_queue, gui_lifecycle)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


def main(args=None) -> None:
    command_queue: Queue[DisplayCommand] = Queue(maxsize=100)
    preload_queue: Queue[PreloadItem | None] = Queue()
    gui_lifecycle = GuiLifecycle(command_queue, preload_queue)

    if DISPLAY_ON_DEMAND:
        Thread(
            target=lambda: AsyncImagePreloader(
                DISPLAY_WIDTH or 1024,
                DISPLAY_HEIGHT or 600,
                preload_queue,
            ).start(),
            daemon=True,
        ).start()
        run_ros(command_queue, gui_lifecycle)
        return

    Thread(target=run_ros, args=(command_queue, gui_lifecycle), daemon=True).start()
    run_gui(command_queue, preload_queue, gui_lifecycle)


if __name__ == "__main__":
    main()
