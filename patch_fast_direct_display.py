from pathlib import Path
import re

ROOT = Path("/root/app/pib-backend")
DISPLAY = ROOT / "ros_packages/display/display/display.py"
PACKAGE = ROOT / "ros_packages/display/package.xml"
COMPOSE = ROOT / "docker-compose.yaml"

DISPLAY.write_text(r'''from queue import Queue, Empty
import base64
from dataclasses import dataclass
from hashlib import sha1
from io import BytesIO
from itertools import cycle
import os
import re
import time
from pathlib import Path
from threading import Thread
from typing import Iterable, Iterator, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import PIL.Image
from PIL import Image as PILImage, ImageDraw, ImageFont
from tkinter import *

from datatypes.msg import DisplayImage, ImageFormat, ImageId
from std_msgs.msg import String


os.environ.setdefault("DISPLAY", ":0.0")

STATIC_IMAGE_DIR = os.getenv(
    "STATIC_IMAGE_DIR",
    "/home/pib/ros_working_dir/src/display/static_images",
)

EXPRESSION_DIR = Path(os.environ.get("PIB_EXPRESSION_DIR", "/app/pib-expression-faces"))
DISPLAY_TEXT_MAX_CHARS = int(os.environ.get("PIB_DISPLAY_TEXT_MAX_CHARS", "40"))
DISPLAY_TEXT_PADDING = int(os.environ.get("PIB_DISPLAY_TEXT_PADDING", "40"))
DISPLAY_POLL_MS = int(os.environ.get("PIB_DISPLAY_POLL_MS", "5"))
DISPLAY_IDLE_SECONDS = float(os.environ.get("PIB_DISPLAY_IDLE_SECONDS", "10"))
DISPLAY_VERBOSE = os.environ.get("PIB_DISPLAY_NODE_VERBOSE", "0") == "1"


def display_debug(message: str):
    if DISPLAY_VERBOSE:
        print(f"[display-node-debug] {time.monotonic():.6f} {message}", flush=True)


@dataclass
class ImageFile:
    format_value: int
    filepath: str


@dataclass
class RawImage:
    format_value: int
    data: bytes

    @staticmethod
    def from_display_image(display_image: DisplayImage):
        image_id = display_image.id.value

        match image_id:
            case ImageId.CUSTOM:
                return RawImage(display_image.format.value, b"".join(display_image.data))
            case ImageId.NONE:
                return None
            case _:
                image_file = IMAGE_ID_TO_STATIC_IMAGES.get(image_id)
                if image_file is None:
                    raise Exception(f"illegal image-id: '{image_id}'.")
                return RawImage.from_image_file(image_file)

    @staticmethod
    def from_image_file(image_file: ImageFile):
        with open(image_file.filepath, "rb") as file:
            data = file.read()
        return RawImage(image_file.format_value, data)


@dataclass
class DisplayCommand:
    kind: str
    value: object = None


@dataclass
class AnimationFrame:
    duration_ms: int
    photo_image: PhotoImage


class Animation:
    def __init__(self, frames: Iterable[AnimationFrame]):
        self._frames = list(frames)
        self._frame_iterator = iter(cycle(self._frames))
        self._stopped = False

    def stop(self) -> None:
        self._stopped = True

    def __iter__(self) -> Iterator[AnimationFrame]:
        return self

    def __next__(self) -> AnimationFrame:
        if self._stopped:
            raise StopIteration()
        return next(self._frame_iterator)


IMAGE_ID_TO_STATIC_IMAGES: dict[int, ImageFile] = {
    ImageId.PIB_EYES_ANIMATED: ImageFile(
        ImageFormat.ANIMATED_GIF,
        STATIC_IMAGE_DIR + "/pib-eyes-animated.gif",
    ),
}

FORMAT_VALUE_TO_STR: dict[int, str] = {
    ImageFormat.ANIMATED_GIF: "gif",
    ImageFormat.PNG: "png",
    ImageFormat.JPEG: "jpeg",
}


class GuiApplication(Frame):
    def __init__(
        self,
        parent: Widget,
        command_queue: Queue[DisplayCommand],
        *args,
        **kwargs,
    ):
        self._width = kwargs.setdefault("width", 100)
        self._height = kwargs.setdefault("height", 100)

        Frame.__init__(self, parent, *args, **kwargs)

        self.command_queue = command_queue
        self.current_main_content: PhotoImage | Animation | None = None
        self.default_timer_id = None

        self.static_image_cache: dict[str, PhotoImage] = {}
        self.animation_frame_cache: dict[str, list[AnimationFrame]] = {}
        self.text_cache: dict[str, PhotoImage] = {}

        self.canvas = Canvas(
            self,
            width=self._width,
            height=self._height,
            borderwidth=0,
            highlightthickness=0,
        )
        self.canvas.place(x=0, y=0)

        self.preload_default()
        self.preload_expressions()
        self.show_default_animation()

        self._poll_next_command()
        self.grid()

    def cache_key_for_raw(self, raw_image: RawImage) -> str:
        return f"{raw_image.format_value}:{sha1(raw_image.data).hexdigest()}"

    def stop_current_animation(self):
        if isinstance(self.current_main_content, Animation):
            self.current_main_content.stop()

    def build_static_photo_image(self, raw_image: RawImage) -> PhotoImage:
        t0 = time.monotonic()
        format_str = FORMAT_VALUE_TO_STR[raw_image.format_value]

        with PIL.Image.open(BytesIO(raw_image.data)) as image:
            resized = image.resize((self._width, self._height))
            data_buffer = BytesIO()
            resized.save(data_buffer, format_str)
            data = base64.b64encode(data_buffer.getvalue())

        photo_image = PhotoImage(data=data, format=format_str)
        display_debug(f"static PhotoImage built in {(time.monotonic() - t0) * 1000:.1f} ms")
        return photo_image

    def build_animation_frames(self, raw_image: RawImage) -> list[AnimationFrame]:
        t0 = time.monotonic()
        frames: list[AnimationFrame] = []

        with PIL.Image.open(BytesIO(raw_image.data)) as image:
            for i in range(image.n_frames):
                image.seek(i)
                resized = image.resize((self._width, self._height))
                data_buffer = BytesIO()
                resized.save(data_buffer, "gif")
                data = base64.b64encode(data_buffer.getvalue())
                duration_ms = image.info.get("duration", 80)
                frames.append(AnimationFrame(duration_ms, PhotoImage(data=data)))

        display_debug(
            f"gif frames built count={len(frames)} in {(time.monotonic() - t0) * 1000:.1f} ms"
        )
        return frames

    def preload_default(self):
        raw = RawImage.from_image_file(IMAGE_ID_TO_STATIC_IMAGES[ImageId.PIB_EYES_ANIMATED])
        key = "default:pib_eyes_animated"
        self.animation_frame_cache[key] = self.build_animation_frames(raw)
        display_debug("default animated eyes preloaded")

    def preload_expressions(self):
        if not EXPRESSION_DIR.exists():
            print(f"[display] expression directory missing: {EXPRESSION_DIR}", flush=True)
            return

        t0 = time.monotonic()
        count = 0

        for path in sorted(EXPRESSION_DIR.iterdir()):
            if not path.is_file():
                continue

            suffix = path.suffix.lower()
            if suffix not in (".png", ".gif", ".jpg", ".jpeg"):
                continue

            expression = path.stem
            raw = RawImage(self.image_format_from_path(path), path.read_bytes())

            if raw.format_value == ImageFormat.ANIMATED_GIF:
                self.animation_frame_cache[f"expression:{expression}"] = self.build_animation_frames(raw)
            else:
                self.static_image_cache[f"expression:{expression}"] = self.build_static_photo_image(raw)

            count += 1

        print(
            f"[display] preloaded {count} expression files in {(time.monotonic() - t0) * 1000:.1f} ms",
            flush=True,
        )

    def image_format_from_path(self, path: Path) -> int:
        suffix = path.suffix.lower()
        if suffix == ".gif":
            return ImageFormat.ANIMATED_GIF
        if suffix == ".png":
            return ImageFormat.PNG
        if suffix in (".jpg", ".jpeg"):
            return ImageFormat.JPEG
        raise RuntimeError(f"unsupported image format: {path}")

    def normalize_expression(self, value: str) -> str:
        value = value.strip().lower()
        value = value.replace("-", "_").replace(" ", "_")
        value = re.sub(r"[^a-z0-9_]", "", value)
        return value

    def show_photo_image(self, photo_image: PhotoImage):
        self.canvas.delete("all")
        self.stop_current_animation()
        self.current_main_content = photo_image
        self.canvas.create_image(0, 0, image=self.current_main_content, anchor="nw")

    def show_animation_frames(self, frames: list[AnimationFrame]):
        self.canvas.delete("all")
        self.stop_current_animation()
        animation = Animation(frames)
        self.current_main_content = animation
        self.show_next_frame(animation)

    def show_next_frame(self, animation: Animation):
        try:
            frame = next(animation)
        except StopIteration:
            return

        self.canvas.delete("all")
        self.canvas.create_image(0, 0, image=frame.photo_image, anchor="nw")
        self.canvas.after(frame.duration_ms, self.show_next_frame, animation)

    def show_default_animation(self):
        frames = self.animation_frame_cache.get("default:pib_eyes_animated")
        if frames is not None:
            self.show_animation_frames(frames)

    def cancel_default_timer(self):
        if self.default_timer_id is not None:
            try:
                self.after_cancel(self.default_timer_id)
            except Exception:
                pass
            self.default_timer_id = None

    def schedule_default_animation(self):
        self.cancel_default_timer()
        self.default_timer_id = self.after(
            int(DISPLAY_IDLE_SECONDS * 1000),
            self.show_default_animation,
        )

    def show_expression(self, expression: str):
        t0 = time.monotonic()
        expression = self.normalize_expression(expression)

        static_key = f"expression:{expression}"
        animation_key = f"expression:{expression}"

        if static_key in self.static_image_cache:
            self.show_photo_image(self.static_image_cache[static_key])
            self.schedule_default_animation()
            print(
                f"[display] Expression shown directly: {expression} total={(time.monotonic() - t0) * 1000:.1f} ms",
                flush=True,
            )
            return

        if animation_key in self.animation_frame_cache:
            self.show_animation_frames(self.animation_frame_cache[animation_key])
            self.schedule_default_animation()
            print(
                f"[display] Expression shown directly: {expression} total={(time.monotonic() - t0) * 1000:.1f} ms",
                flush=True,
            )
            return

        print(f"[display] expression not found in cache: {expression}", flush=True)

    def load_font(self, size: int):
        candidates = [
            "/usr/share/fonts/truetype/msttcorefonts/Arial_Rounded_MT_Bold.ttf",
            "/usr/share/fonts/truetype/msttcorefonts/Arial.ttf",
            "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
            "/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf",
        ]
        for candidate in candidates:
            try:
                return ImageFont.truetype(candidate, size)
            except Exception:
                pass
        return ImageFont.load_default()

    def wrap_text_for_font(self, draw, value: str, font, max_width: int):
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

    def build_text_photo_image(self, value: str) -> PhotoImage:
        value = str(value).strip()[:DISPLAY_TEXT_MAX_CHARS]
        width = 800
        height = 480
        padding = DISPLAY_TEXT_PADDING

        image = PILImage.new("RGBA", (width, height), (0, 0, 0, 255))
        draw = ImageDraw.Draw(image)
        color = (69, 183, 255, 255)

        best_font = self.load_font(20)
        best_lines = [value]

        for size in range(180, 18, -4):
            font = self.load_font(size)
            lines = self.wrap_text_for_font(draw, value, font, width - 2 * padding)
            boxes = [draw.textbbox((0, 0), line, font=font) for line in lines]
            line_heights = [box[3] - box[1] for box in boxes]
            total_height = sum(line_heights) + max(0, len(lines) - 1) * int(size * 0.25)
            max_line_width = max((box[2] - box[0]) for box in boxes) if boxes else 0

            if max_line_width <= width - 2 * padding and total_height <= height - 2 * padding:
                best_font = font
                best_lines = lines
                break

        boxes = [draw.textbbox((0, 0), line, font=best_font) for line in best_lines]
        line_heights = [box[3] - box[1] for box in boxes]
        spacing = 16
        total_height = sum(line_heights) + max(0, len(best_lines) - 1) * spacing
        y = (height - total_height) // 2

        for line, box, line_height in zip(best_lines, boxes, line_heights):
            line_width = box[2] - box[0]
            x = (width - line_width) // 2
            draw.text((x, y), line, font=best_font, fill=color)
            y += line_height + spacing

        data_buffer = BytesIO()
        image.save(data_buffer, "png")
        data = base64.b64encode(data_buffer.getvalue())
        return PhotoImage(data=data, format="png")

    def show_text(self, value: str):
        t0 = time.monotonic()
        value = str(value).strip()[:DISPLAY_TEXT_MAX_CHARS]

        photo_image = self.text_cache.get(value)
        if photo_image is None:
            photo_image = self.build_text_photo_image(value)
            self.text_cache[value] = photo_image

        self.show_photo_image(photo_image)
        self.schedule_default_animation()

        print(
            f"[display] Display text shown directly: {value} total={(time.monotonic() - t0) * 1000:.1f} ms",
            flush=True,
        )

    def show_raw_image(self, raw_image: RawImage):
        if raw_image.format_value == ImageFormat.ANIMATED_GIF:
            key = self.cache_key_for_raw(raw_image)
            frames = self.animation_frame_cache.get(key)
            if frames is None:
                frames = self.build_animation_frames(raw_image)
                self.animation_frame_cache[key] = frames
            self.show_animation_frames(frames)
            return

        key = self.cache_key_for_raw(raw_image)
        photo_image = self.static_image_cache.get(key)
        if photo_image is None:
            photo_image = self.build_static_photo_image(raw_image)
            self.static_image_cache[key] = photo_image
        self.show_photo_image(photo_image)

    def handle_command(self, command: DisplayCommand):
        match command.kind:
            case "expression":
                self.show_expression(str(command.value))
            case "text":
                self.show_text(str(command.value))
            case "raw_image":
                self.show_raw_image(command.value)
            case "default":
                self.show_default_animation()
            case "quit":
                self.winfo_toplevel().destroy()

    def _poll_next_command(self):
        try:
            while True:
                command = self.command_queue.get_nowait()
                self.handle_command(command)
        except Empty:
            pass

        self.after(DISPLAY_POLL_MS, self._poll_next_command)


class DisplayNode(Node):
    def __init__(self, command_queue: Queue[DisplayCommand]) -> None:
        super().__init__("display")
        self.command_queue = command_queue

        self.display_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            DisplayImage,
            "/display_image",
            self.on_display_image_received,
            self.display_qos,
        )

        self.create_subscription(
            String,
            "/pib/expression",
            self.on_expression_received,
            10,
        )

        self.create_subscription(
            String,
            "/pib/display_text",
            self.on_display_text_received,
            10,
        )

        self.get_logger().info("Now Running DISPLAY")
        self.get_logger().info(f"Expression directory: {EXPRESSION_DIR}")

    def enqueue_latest(self, command: DisplayCommand):
        while not self.command_queue.empty():
            try:
                self.command_queue.get_nowait()
            except Exception:
                break
        self.command_queue.put(command)

    def on_display_image_received(self, display_image: DisplayImage):
        try:
            raw_image = RawImage.from_display_image(display_image)
            if raw_image is not None:
                self.enqueue_latest(DisplayCommand("raw_image", raw_image))
        except Exception as e:
            self.get_logger().error(f"error while showing image from topic: {e}.")

    def on_expression_received(self, msg: String):
        self.enqueue_latest(DisplayCommand("expression", msg.data))

    def on_display_text_received(self, msg: String):
        self.enqueue_latest(DisplayCommand("text", msg.data))


def run_gui_application(command_queue: Queue[DisplayCommand]) -> None:
    root = Tk()
    root.bind("<Escape>", lambda _: root.destroy())
    root.attributes("-fullscreen", True)

    width = root.winfo_screenwidth()
    height = root.winfo_screenheight()

    GuiApplication(root, command_queue, width=width, height=height)
    root.mainloop()


def run_display_node(command_queue: Queue[DisplayCommand]) -> None:
    rclpy.init()
    executor = SingleThreadedExecutor()
    display_node = DisplayNode(command_queue)
    executor.add_node(display_node)
    executor.spin()
    display_node.destroy_node()
    rclpy.shutdown()


def main(args=None) -> None:
    command_queue: Queue[DisplayCommand] = Queue(maxsize=1)
    Thread(daemon=True, target=run_display_node, args=(command_queue,)).start()
    run_gui_application(command_queue)


if __name__ == "__main__":
    main()
''')


print("fast direct display.py written")
