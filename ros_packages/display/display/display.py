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
from threading import Thread
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


os.environ.setdefault("DISPLAY", ":0.0")

STATIC_IMAGE_DIR = Path(os.getenv("STATIC_IMAGE_DIR", "/app/ros2_ws/display/static_images"))
EXPRESSION_DIR = Path(os.getenv("PIB_EXPRESSION_DIR", "/app/pib-expression-faces"))

DISPLAY_TEXT_MAX_CHARS = int(os.getenv("PIB_DISPLAY_TEXT_MAX_CHARS", "40"))
DISPLAY_TEXT_PADDING = int(os.getenv("PIB_DISPLAY_TEXT_PADDING", "40"))
DISPLAY_POLL_MS = int(os.getenv("PIB_DISPLAY_POLL_MS", "5"))
DISPLAY_IDLE_SECONDS = float(os.getenv("PIB_DISPLAY_IDLE_SECONDS", "10"))
DISPLAY_VERBOSE = os.getenv("PIB_DISPLAY_NODE_VERBOSE", "0") == "1"
DISPLAY_TEXT_CACHE_MAX = int(os.getenv("PIB_DISPLAY_TEXT_CACHE_MAX", "50"))

TEXT_CANVAS_WIDTH = int(os.getenv("PIB_DISPLAY_TEXT_WIDTH", "800"))
TEXT_CANVAS_HEIGHT = int(os.getenv("PIB_DISPLAY_TEXT_HEIGHT", "480"))


def log_debug(message: str) -> None:
    if DISPLAY_VERBOSE:
        print(f"[display-v2-debug] {time.monotonic():.6f} {message}", flush=True)


def log_info(message: str) -> None:
    print(f"[display-v2] {message}", flush=True)


@dataclass(frozen=True)
class RawImage:
    format_value: int
    data: bytes


@dataclass
class AnimationFrame:
    duration_ms: int
    photo_image: PhotoImage


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


class ImageCache:
    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height
        self.static_images: dict[str, PhotoImage] = {}
        self.animations: dict[str, list[AnimationFrame]] = {}

    def preload_default_eyes(self) -> None:
        path = STATIC_IMAGE_DIR / "pib-eyes-animated.gif"
        if not path.exists():
            raise FileNotFoundError(f"default eyes missing: {path}")

        raw = RawImage(ImageFormat.ANIMATED_GIF, path.read_bytes())
        self.animations["default"] = self._build_animation_frames(raw)
        log_info("default animated eyes preloaded")

    def preload_expressions(self) -> None:
        if not EXPRESSION_DIR.exists():
            log_info(f"expression directory missing: {EXPRESSION_DIR}")
            return

        start = time.monotonic()
        count = 0

        for path in sorted(EXPRESSION_DIR.iterdir()):
            if not path.is_file() or path.suffix.lower() not in (".png", ".gif", ".jpg", ".jpeg"):
                continue

            expression = self.normalize_expression(path.stem)
            raw = RawImage(self.format_from_path(path), path.read_bytes())

            if raw.format_value == ImageFormat.ANIMATED_GIF:
                self.animations[f"expression:{expression}"] = self._build_animation_frames(raw)
            else:
                self.static_images[f"expression:{expression}"] = self._build_static_photo(raw)

            count += 1

        log_info(f"preloaded {count} expression files in {(time.monotonic() - start) * 1000:.1f} ms")

    def get_expression(self, expression: str) -> tuple[str, object] | None:
        expression = self.normalize_expression(expression)
        key = f"expression:{expression}"

        if key in self.static_images:
            return ("static", self.static_images[key])

        if key in self.animations:
            return ("animation", self.animations[key])

        return None

    def get_default_animation(self) -> list[AnimationFrame]:
        return self.animations["default"]

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

    def _build_static_photo(self, raw: RawImage) -> PhotoImage:
        start = time.monotonic()
        format_str = self.format_to_str(raw.format_value)

        with PIL.Image.open(BytesIO(raw.data)) as image:
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
                resized = image.resize((self.width, self.height))

                buffer = BytesIO()
                resized.save(buffer, "gif")
                data = base64.b64encode(buffer.getvalue())
                duration_ms = image.info.get("duration", 80)

                frames.append(AnimationFrame(duration_ms, PhotoImage(data=data)))

        log_debug(f"animation built frames={len(frames)} in {(time.monotonic() - start) * 1000:.1f} ms")
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

    def show_default(self) -> None:
        self.cancel_default_timer()
        self.show_animation(self.image_cache.get_default_animation())

    def show_expression(self, expression: str) -> None:
        start = time.monotonic()
        result = self.image_cache.get_expression(expression)

        if result is None:
            log_info(f"expression not found: {expression}")
            return

        kind, payload = result
        self.show_payload(kind, payload)
        self.schedule_default()

        log_info(f"expression shown: {expression} total={(time.monotonic() - start) * 1000:.1f} ms")

    def show_text(self, value: str) -> None:
        start = time.monotonic()
        photo = self.text_renderer.render(value)
        self.show_photo(photo)
        self.schedule_default()
        log_info(f"text shown: {str(value)[:DISPLAY_TEXT_MAX_CHARS]} total={(time.monotonic() - start) * 1000:.1f} ms")

    def show_raw(self, raw: RawImage) -> None:
        kind, payload = self.image_cache.get_raw_image(raw)
        self.show_payload(kind, payload)

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

    def schedule_default(self) -> None:
        self.cancel_default_timer()
        self.default_timer_id = self.canvas.after(int(DISPLAY_IDLE_SECONDS * 1000), self.show_default)

    def cancel_default_timer(self) -> None:
        if self.default_timer_id is not None:
            try:
                self.canvas.after_cancel(self.default_timer_id)
            except Exception:
                pass
            self.default_timer_id = None


class GuiApplication(Frame):
    def __init__(self, parent: Widget, command_queue: Queue[DisplayCommand], *args, **kwargs):
        self.width = kwargs.setdefault("width", 100)
        self.height = kwargs.setdefault("height", 100)

        Frame.__init__(self, parent, *args, **kwargs)

        self.command_queue = command_queue

        self.canvas = Canvas(
            self,
            width=self.width,
            height=self.height,
            borderwidth=0,
            highlightthickness=0,
        )
        self.canvas.place(x=0, y=0)

        self.image_cache = ImageCache(self.width, self.height)
        self.text_renderer = TextRenderer(self.width, self.height)
        self.controller = DisplayController(self.canvas, parent, self.image_cache, self.text_renderer)

        self.image_cache.preload_default_eyes()
        self.image_cache.preload_expressions()
        self.controller.show_default()

        self._poll_commands()
        self.grid()

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
            self.controller.show_expression(str(command.value))
        elif command.kind == "text":
            self.controller.show_text(str(command.value))
        elif command.kind == "raw":
            self.controller.show_raw(command.value)
        elif command.kind == "default":
            self.controller.show_default()
        elif command.kind == "quit":
            self.winfo_toplevel().destroy()


class DisplayNode(Node):
    def __init__(self, command_queue: Queue[DisplayCommand]) -> None:
        super().__init__("display")
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

        self.get_logger().info("Display V2 running")
        self.get_logger().info(f"Expression directory: {EXPRESSION_DIR}")

    def enqueue_latest(self, command: DisplayCommand) -> None:
        self.command_queue.put(command)

    def on_expression(self, msg: String) -> None:
        self.enqueue_latest(DisplayCommand("expression", msg.data))

    def on_text(self, msg: String) -> None:
        self.enqueue_latest(DisplayCommand("text", msg.data))

    def on_display_image(self, msg: DisplayImage) -> None:
        try:
            raw = self.raw_from_display_image(msg)
            if raw is not None:
                self.enqueue_latest(DisplayCommand("raw", raw))
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


def run_gui(command_queue: Queue[DisplayCommand]) -> None:
    root = Tk()
    root.bind("<Escape>", lambda _: root.destroy())
    root.attributes("-fullscreen", True)

    width = root.winfo_screenwidth()
    height = root.winfo_screenheight()

    GuiApplication(root, command_queue, width=width, height=height)
    root.mainloop()


def run_ros(command_queue: Queue[DisplayCommand]) -> None:
    rclpy.init()
    executor = SingleThreadedExecutor()
    node = DisplayNode(command_queue)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


def main(args=None) -> None:
    command_queue: Queue[DisplayCommand] = Queue(maxsize=100)
    Thread(daemon=True, target=run_ros, args=(command_queue,)).start()
    run_gui(command_queue)


if __name__ == "__main__":
    main()
