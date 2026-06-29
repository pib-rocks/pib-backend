import os
import time
from array import array
import re
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from datatypes.msg import DisplayImage
from PIL import Image, ImageDraw, ImageFont
import io


class PibExpressionManager(Node):
    def __init__(self):
        super().__init__("pib_expression_manager")

        self.expression_dir = Path(
            os.environ.get("PIB_EXPRESSION_DIR", "/app/pib-expression-faces")
        )
        self.verbose_display = os.environ.get("PIB_DISPLAY_VERBOSE", "0") == "1"

        self.display_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.publisher = self.create_publisher(DisplayImage, "/display_image", self.display_qos)
        self.expression_cache = {}
        self.auto_return_seconds = float(os.environ.get("PIB_EXPRESSION_TIMEOUT", "15"))
        self.last_expression_time = time.monotonic()
        self.default_is_active = True
        self.create_timer(0.1, self.on_timer)

        self.subscription = self.create_subscription(
            String,
            "/pib/expression",
            self.on_expression,
            10,
        )

        self.text_subscription = self.create_subscription(
            String,
            "/pib/display_text",
            self.on_display_text,
            10,
        )

        self.get_logger().info("PIB expression manager started")
        self.get_logger().info(f"Expression directory: {self.expression_dir}")

    def debug_log(self, message: str):
        if self.verbose_display:
            self.get_logger().info("[display-debug] " + message)

    def normalize_expression(self, value: str) -> str:
        value = value.strip().lower()
        value = value.replace("-", "_").replace(" ", "_")
        value = re.sub(r"[^a-z0-9_]", "", value)
        return value

    def find_expression_file(self, expression: str) -> Path:
        for suffix in (".png", ".gif", ".jpg", ".jpeg"):
            candidate = self.expression_dir / f"{expression}{suffix}"
            if candidate.exists():
                return candidate

        raise FileNotFoundError(
            f"Expression file not found for '{expression}' in {self.expression_dir}. "
            f"Expected {expression}.png, {expression}.gif, {expression}.jpg or {expression}.jpeg"
        )

    def get_image_format(self, path: Path) -> int:
        suffix = path.suffix.lower()

        if suffix == ".gif":
            return 0  # ANIMATED_GIF
        if suffix == ".png":
            return 1  # PNG
        if suffix in (".jpg", ".jpeg"):
            return 2  # JPEG

        raise RuntimeError(f"Unsupported image format: {path}")

    def get_cached_expression_message(self, path: Path):
        cache_key = str(path)

        cached = self.expression_cache.get(cache_key)
        if cached is not None:
            return cached

        msg = DisplayImage()
        msg.id.value = 1  # CUSTOM
        msg.format.value = self.get_image_format(path)
        msg.data = [bytes([b]) for b in path.read_bytes()]

        self.expression_cache[cache_key] = msg
        self.get_logger().info(f"Cached expression image: {path.name}")
        return msg

    def publish_expression_file(self, path: Path):
        t0 = time.monotonic()
        msg = self.get_cached_expression_message(path)
        self.debug_log(f"message ready for '{path.name}' in {(time.monotonic() - t0) * 1000:.1f} ms")
        self.publisher.publish(msg)
        self.debug_log(f"ros publish called for '{path.name}'")

    def show_default_animation(self):
        msg = DisplayImage()
        msg.id.value = 2       # PIB_EYES_ANIMATED
        msg.format.value = 0   # ANIMATED_GIF
        msg.data = []

        self.publisher.publish(msg)
        self.default_is_active = True
        self.get_logger().info("Default PIB animated eyes shown")

    def on_timer(self):
        if self.auto_return_seconds <= 0:
            return

        if self.default_is_active:
            return

        elapsed = time.monotonic() - self.last_expression_time
        if elapsed >= self.auto_return_seconds:
            self.show_default_animation()


    def publish_png_bytes(self, data: bytes):
        msg = DisplayImage()
        msg.id.value = 1
        msg.format.value = 1
        msg.data = [bytes([b]) for b in data]
        self.publisher.publish(msg)

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

    def render_text_png(self, value: str) -> bytes:
        value = str(value).strip()
        max_chars = int(os.environ.get("PIB_DISPLAY_TEXT_MAX_CHARS", "40"))
        value = value[:max_chars]

        width = int(os.environ.get("PIB_DISPLAY_WIDTH", "800"))
        height = int(os.environ.get("PIB_DISPLAY_HEIGHT", "480"))
        padding = int(os.environ.get("PIB_DISPLAY_TEXT_PADDING", "40"))

        image = Image.new("RGBA", (width, height), (0, 0, 0, 255))
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

        buffer = io.BytesIO()
        image.save(buffer, format="PNG")
        return buffer.getvalue()

    def on_display_text(self, msg: String):
        t0 = time.monotonic()
        raw = msg.data
        self.debug_log(f"display_text received chars={len(raw)} text='{raw[:40]}'")

        try:
            self.last_expression_time = time.monotonic()
            self.default_is_active = False

            t_render = time.monotonic()
            png = self.render_text_png(raw)
            self.debug_log(f"text rendered png_bytes={len(png)} in {(time.monotonic() - t_render) * 1000:.1f} ms")

            t_publish = time.monotonic()
            self.publish_png_bytes(png)
            self.debug_log(f"text published in {(time.monotonic() - t_publish) * 1000:.1f} ms")

            self.get_logger().info(f"Display text shown: {raw[:40]} total={(time.monotonic() - t0) * 1000:.1f} ms")
        except Exception as exc:
            self.get_logger().error(f"Could not show display text: {exc}")


    def on_expression(self, msg: String):
        t0 = time.monotonic()
        raw = msg.data
        expression = self.normalize_expression(raw)

        self.debug_log(f"expression received raw='{raw}' normalized='{expression}'")

        # Kein Cooldown: Jede neue Expression wird sofort verarbeitet.
        self.last_expression_time = time.monotonic()
        self.default_is_active = False

        try:
            t_find = time.monotonic()
            path = self.find_expression_file(expression)
            self.debug_log(f"file resolved expression='{expression}' file='{path}' in {(time.monotonic() - t_find) * 1000:.1f} ms")

            t_pub = time.monotonic()
            self.publish_expression_file(path)
            self.debug_log(f"published expression='{expression}' in {(time.monotonic() - t_pub) * 1000:.1f} ms")

            self.get_logger().info(
                f"Expression shown: {expression} -> {path.name} total={(time.monotonic() - t0) * 1000:.1f} ms"
            )
        except Exception as exc:
            self.get_logger().error(str(exc))


def main(args=None):
    rclpy.init(args=args)
    node = PibExpressionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
