from __future__ import annotations

import os
import re
import time
from dataclasses import dataclass
from hashlib import sha1
from io import BytesIO
from pathlib import Path
from queue import Empty, Queue
from threading import Event as ThreadEvent, Lock, Thread
from typing import Optional

import PIL.Image
import pygame
import rclpy
from PIL import Image as PILImage, ImageDraw, ImageFont
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from datatypes.msg import DisplayImage, ImageFormat, ImageId


STATIC_IMAGE_DIR = Path(os.getenv("STATIC_IMAGE_DIR", "/app/ros2_ws/display/static_images"))
EXPRESSION_DIR = Path(os.getenv("PIB_EXPRESSION_DIR", "/app/pib-expression-faces"))

DISPLAY_WIDTH = int(os.getenv("PIB_DISPLAY_WIDTH", "1024"))
DISPLAY_HEIGHT = int(os.getenv("PIB_DISPLAY_HEIGHT", "600"))
DISPLAY_TEXT_MAX_CHARS = int(os.getenv("PIB_DISPLAY_TEXT_MAX_CHARS", "40"))
DISPLAY_TEXT_PADDING = int(os.getenv("PIB_DISPLAY_TEXT_PADDING", "40"))
DISPLAY_ON_DEMAND = os.getenv("PIB_DISPLAY_ON_DEMAND", "1") == "1"
DISPLAY_VERBOSE = os.getenv("PIB_DISPLAY_NODE_VERBOSE", "0") == "1"
DISPLAY_TEXT_CACHE_MAX = int(os.getenv("PIB_DISPLAY_TEXT_CACHE_MAX", "50"))

TEXT_CANVAS_WIDTH = int(os.getenv("PIB_DISPLAY_TEXT_WIDTH", str(DISPLAY_WIDTH)))
TEXT_CANVAS_HEIGHT = int(os.getenv("PIB_DISPLAY_TEXT_HEIGHT", str(DISPLAY_HEIGHT)))


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


@dataclass(frozen=True)
class PreloadItem:
    cache_key: str
    kind: str  # "static" | "animation"
    width: int
    height: int
    frames: list[tuple[int, bytes]]  # [(duration_ms, rgba_bytes)]


@dataclass(frozen=True)
class DisplayCommand:
    kind: str  # "expression" | "text" | "raw" | "hide" | "quit"
    value: object = None


class SurfaceCache:
    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height
        self.static_surfaces: dict[str, pygame.Surface] = {}
        self.animations: dict[str, list[tuple[int, pygame.Surface]]] = {}

    def apply_preload_item(self, item: PreloadItem) -> None:
        if item.kind == "static":
            _dur, rgba = item.frames[0]
            self.static_surfaces[item.cache_key] = surface_from_rgba(rgba, item.width, item.height)
            return

        self.animations[item.cache_key] = [
            (duration_ms, surface_from_rgba(rgba, item.width, item.height))
            for duration_ms, rgba in item.frames
        ]

    def lazy_load_expression(self, expression: str) -> tuple[str, object] | None:
        path = self._find_expression_path(expression)
        if path is None:
            return None

        normalized = self.normalize_expression(expression)
        key = f"expression:{normalized}"
        suffix = path.suffix.lower()

        if suffix == ".gif":
            with PIL.Image.open(path) as image:
                frames: list[tuple[int, pygame.Surface]] = []
                for index in range(image.n_frames):
                    image.seek(index)
                    frame = image.convert("RGBA")
                    if frame.size != (self.width, self.height):
                        frame = frame.resize((self.width, self.height), PIL.Image.LANCZOS)
                    duration_ms = int(image.info.get("duration", 80))
                    frames.append((duration_ms, surface_from_rgba(frame.tobytes(), self.width, self.height)))
            self.animations[key] = frames
            return ("animation", frames)

        with PIL.Image.open(path) as image:
            frame = image.convert("RGBA")
            if frame.size != (self.width, self.height):
                frame = frame.resize((self.width, self.height), PIL.Image.LANCZOS)
            surf = surface_from_rgba(frame.tobytes(), self.width, self.height)
        self.static_surfaces[key] = surf
        return ("static", surf)

    def _find_expression_path(self, expression: str) -> Path | None:
        if not EXPRESSION_DIR.exists():
            return None
        normalized = self.normalize_expression(expression)
        for path in sorted(EXPRESSION_DIR.iterdir()):
            if not path.is_file() or path.suffix.lower() not in (".png", ".gif", ".jpg", ".jpeg"):
                continue
            if self.normalize_expression(path.stem) == normalized:
                return path
        return None

    @staticmethod
    def normalize_expression(value: str) -> str:
        value = value.strip().lower()
        value = value.replace("-", "_").replace(" ", "_")
        return re.sub(r"[^a-z0-9_]", "", value)

    def get_expression(self, expression: str) -> tuple[str, object] | None:
        expression = self.normalize_expression(expression)
        key = f"expression:{expression}"

        if key in self.static_surfaces:
            return ("static", self.static_surfaces[key])

        if key in self.animations:
            return ("animation", self.animations[key])

        return None

    def get_default_animation(self) -> list[tuple[int, pygame.Surface]] | None:
        return self.animations.get("default")

    def get_raw_image(self, raw: RawImage) -> tuple[str, object]:
        key = f"raw:{raw.format_value}:{sha1(raw.data).hexdigest()}"

        if raw.format_value == ImageFormat.ANIMATED_GIF:
            if key not in self.animations:
                self.animations[key] = self._build_animation_surfaces(raw)
            return ("animation", self.animations[key])

        if key not in self.static_surfaces:
            self.static_surfaces[key] = self._build_static_surface(raw)
        return ("static", self.static_surfaces[key])

    def _build_static_surface(self, raw: RawImage) -> pygame.Surface:
        with PIL.Image.open(BytesIO(raw.data)) as image:
            frame = image.convert("RGBA")
            if frame.size != (self.width, self.height):
                frame = frame.resize((self.width, self.height), PIL.Image.LANCZOS)
            return surface_from_rgba(frame.tobytes(), self.width, self.height)

    def _build_animation_surfaces(self, raw: RawImage) -> list[tuple[int, pygame.Surface]]:
        frames: list[tuple[int, pygame.Surface]] = []
        with PIL.Image.open(BytesIO(raw.data)) as image:
            for index in range(image.n_frames):
                image.seek(index)
                frame = image.convert("RGBA")
                if frame.size != (self.width, self.height):
                    frame = frame.resize((self.width, self.height), PIL.Image.LANCZOS)
                duration_ms = int(image.info.get("duration", 80))
                frames.append((duration_ms, surface_from_rgba(frame.tobytes(), self.width, self.height)))
        return frames


def surface_from_rgba(rgba: bytes, width: int, height: int) -> pygame.Surface:
    # Avoid convert_alpha(): it requires an initialized display surface.
    # The returned Surface is still blit-able once the window exists.
    return pygame.image.frombuffer(rgba, (width, height), "RGBA")


class AsyncImagePreloader:
    def __init__(self, width: int, height: int, result_queue: Queue[PreloadItem | None]):
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

            expression = SurfaceCache.normalize_expression(path.stem)
            key = f"expression:{expression}"
            kind = "animation" if path.suffix.lower() == ".gif" else "static"
            self.result_queue.put(self._build_preload_item(key, kind, path))
            count += 1

        log_info(f"preloaded {count} expression files in {(time.monotonic() - start) * 1000:.1f} ms (background)")

    def _build_preload_item(self, cache_key: str, kind: str, path: Path) -> PreloadItem:
        if kind == "static":
            frames = [(0, self._encode_static_rgba(path))]
            return PreloadItem(cache_key, kind, self.width, self.height, frames)

        return PreloadItem(cache_key, kind, self.width, self.height, self._encode_animation_rgba(path))

    def _encode_static_rgba(self, path: Path) -> bytes:
        with PIL.Image.open(path) as image:
            frame = image.convert("RGBA")
            if frame.size != (self.width, self.height):
                frame = frame.resize((self.width, self.height), PIL.Image.LANCZOS)
            return frame.tobytes()

    def _encode_animation_rgba(self, path: Path) -> list[tuple[int, bytes]]:
        frames: list[tuple[int, bytes]] = []
        with PIL.Image.open(path) as image:
            for index in range(image.n_frames):
                image.seek(index)
                frame = image.convert("RGBA")
                if frame.size != (self.width, self.height):
                    frame = frame.resize((self.width, self.height), PIL.Image.LANCZOS)
                duration_ms = int(image.info.get("duration", 80))
                frames.append((duration_ms, frame.tobytes()))
        return frames


class TextRenderer:
    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height
        self.cache: dict[str, pygame.Surface] = {}
        self.cache_order: list[str] = []

    def render(self, value: str) -> pygame.Surface:
        value = str(value).strip()[:DISPLAY_TEXT_MAX_CHARS]
        if value in self.cache:
            return self.cache[value]

        surface = self._build_text_surface(value)
        self.cache[value] = surface
        self.cache_order.append(value)
        while len(self.cache_order) > DISPLAY_TEXT_CACHE_MAX:
            old = self.cache_order.pop(0)
            self.cache.pop(old, None)
        return surface

    def _build_text_surface(self, value: str) -> pygame.Surface:
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

        rgba = image.tobytes()
        return surface_from_rgba(rgba, self.width, self.height)

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


class WaylandRenderer:
    def __init__(self, command_queue: Queue[DisplayCommand], preload_queue: Queue[PreloadItem | None]):
        self.command_queue = command_queue
        self.preload_queue = preload_queue
        self.cache = SurfaceCache(DISPLAY_WIDTH, DISPLAY_HEIGHT)
        self.text_renderer = TextRenderer(TEXT_CANVAS_WIDTH, TEXT_CANVAS_HEIGHT)

        self.window_visible = not DISPLAY_ON_DEMAND
        self.screen: Optional[pygame.Surface] = None
        self.running = True

        self.current_kind: str | None = None
        self.current_static: Optional[pygame.Surface] = None
        self.current_animation: Optional[list[tuple[int, pygame.Surface]]] = None
        self.anim_index = 0
        self.next_frame_at_ms = 0

        self.preload_done = False

    def init_display(self) -> None:
        pygame.init()
        pygame.display.init()
        pygame.event.set_allowed([pygame.QUIT, pygame.KEYDOWN, pygame.MOUSEBUTTONDOWN])

        if DISPLAY_ON_DEMAND:
            return

        self._ensure_window()

    def _ensure_window(self) -> None:
        if self.screen is not None:
            return

        flags = pygame.FULLSCREEN
        self.screen = pygame.display.set_mode((DISPLAY_WIDTH, DISPLAY_HEIGHT), flags)
        pygame.display.set_caption("pib-display")
        self.window_visible = True
        self._render_black()

    def _hide_window(self) -> None:
        if self.screen is None:
            return
        pygame.display.quit()
        self.screen = None
        self.window_visible = False
        log_info("display dismissed by user")

    def _render_black(self) -> None:
        if self.screen is None:
            return
        self.screen.fill((0, 0, 0))
        pygame.display.flip()

    def _apply_preload(self) -> None:
        try:
            while True:
                item = self.preload_queue.get_nowait()
                if item is None:
                    self.preload_done = True
                    return
                self.cache.apply_preload_item(item)
        except Empty:
            return

    def _start_animation(self, frames: list[tuple[int, pygame.Surface]]) -> None:
        self.current_kind = "animation"
        self.current_animation = frames
        self.current_static = None
        self.anim_index = 0
        self.next_frame_at_ms = pygame.time.get_ticks()

    def _set_static(self, surf: pygame.Surface) -> None:
        self.current_kind = "static"
        self.current_static = surf
        self.current_animation = None

    def _draw(self) -> None:
        if self.screen is None:
            return

        if self.current_kind is None:
            self._render_black()
            return

        if self.current_kind == "static" and self.current_static is not None:
            self.screen.blit(self.current_static, (0, 0))
            pygame.display.flip()
            return

        if self.current_kind == "animation" and self.current_animation:
            now = pygame.time.get_ticks()
            if now >= self.next_frame_at_ms:
                duration_ms, frame = self.current_animation[self.anim_index]
                self.screen.blit(frame, (0, 0))
                pygame.display.flip()
                self.next_frame_at_ms = now + max(1, int(duration_ms))
                self.anim_index = (self.anim_index + 1) % len(self.current_animation)

    def _handle_command(self, cmd: DisplayCommand) -> None:
        if cmd.kind in ("expression", "text", "raw"):
            if DISPLAY_ON_DEMAND:
                self._ensure_window()
                # Drain preload queue now that a display exists.
                self._apply_preload()

        if cmd.kind == "hide":
            if DISPLAY_ON_DEMAND:
                self._hide_window()
            else:
                self._render_black()
            return

        if cmd.kind == "quit":
            self.running = False
            return

        if cmd.kind == "expression":
            start = time.monotonic()
            expression = str(cmd.value)
            result = self.cache.get_expression(expression)
            if result is None:
                log_info(f"expression not ready yet: {expression}")
                result = self.cache.lazy_load_expression(expression)
                if result is None:
                    log_info(f"expression not found: {expression}")
                    return
            kind, payload = result
            if kind == "static":
                self._set_static(payload)
            else:
                self._start_animation(payload)
            log_info(f"expression shown: {expression} total={(time.monotonic() - start) * 1000:.1f} ms")
            return

        if cmd.kind == "text":
            start = time.monotonic()
            value = str(cmd.value)
            surf = self.text_renderer.render(value)
            self._set_static(surf)
            log_info(f"text shown: {value[:DISPLAY_TEXT_MAX_CHARS]} total={(time.monotonic() - start) * 1000:.1f} ms")
            return

        if cmd.kind == "raw":
            kind, payload = self.cache.get_raw_image(cmd.value)
            if kind == "static":
                self._set_static(payload)
            else:
                self._start_animation(payload)

    def loop(self, ready_evt: ThreadEvent | None = None) -> None:
        self.init_display()
        if ready_evt is not None:
            ready_evt.set()

        clock = pygame.time.Clock()

        while self.running:
            # In on-demand mode we only create a display after the first trigger.
            # Converting surfaces before a display exists can crash on some pygame backends.
            if self.screen is not None:
                self._apply_preload()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    if DISPLAY_ON_DEMAND:
                        self._hide_window()
                    else:
                        self._render_black()
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if DISPLAY_ON_DEMAND:
                        self._hide_window()
                    else:
                        self._render_black()

            try:
                while True:
                    cmd = self.command_queue.get_nowait()
                    self._handle_command(cmd)
            except Empty:
                pass

            self._draw()
            clock.tick(60)

        pygame.quit()


def run_renderer(
    command_queue: Queue[DisplayCommand],
    preload_queue: Queue[PreloadItem | None],
    lifecycle=None,
) -> None:
    ready_evt = None
    renderer = WaylandRenderer(command_queue, preload_queue)
    renderer.loop(ready_evt=ready_evt)


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

        ready_msg = String()
        ready_msg.data = "ready"
        self.ready_pub.publish(ready_msg)

        self.get_logger().info("Display V2 running (Wayland)")
        self.get_logger().info(f"Expression directory: {EXPRESSION_DIR}")
        if DISPLAY_ON_DEMAND:
            self.get_logger().info("On-demand mode: window opens only when triggered")
        self.get_logger().info("published /pib/display_ready")

    def enqueue(self, command: DisplayCommand) -> None:
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


def run_ros(command_queue: Queue[DisplayCommand]) -> None:
    rclpy.init()
    executor = SingleThreadedExecutor()
    node = DisplayNode(command_queue)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


def main(args=None) -> None:
    command_queue: Queue[DisplayCommand] = Queue(maxsize=200)
    preload_queue: Queue[PreloadItem | None] = Queue()

    AsyncImagePreloader(DISPLAY_WIDTH, DISPLAY_HEIGHT, preload_queue).start()

    # SDL/Wayland is safest when the renderer runs in the main thread.
    Thread(target=run_ros, args=(command_queue,), daemon=True).start()
    run_renderer(command_queue, preload_queue, None)


if __name__ == "__main__":
    main()

