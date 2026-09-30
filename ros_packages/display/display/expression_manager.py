import os
import time
from array import array
import re
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, String
from datatypes.msg import ChatIsListening, DisplayImage, VoiceAssistantState
from PIL import Image, ImageDraw, ImageFont
import io

from pib_hermes_config.visible_state import (
    PHASE_IDLE,
    cached_key_store_mode,
    resolve_visible_state,
)


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

        self.publisher = self.create_publisher(
            DisplayImage, "/display_image", self.display_qos
        )
        self.expression_cache = {}
        self.auto_return_seconds = float(os.environ.get("PIB_EXPRESSION_TIMEOUT", "15"))
        self.last_expression_time = time.monotonic()
        self.default_is_active = True
        # Hardware VAD drives the face only while no conversation phase is showing.
        self._hardware_vad_active = False
        self._applied_phase = None
        self._applied_text = None
        self._operating_mode = None
        self._voice_turned_on = False
        self._voice_chat_id = ""
        self._personality_id = ""
        self._personality_name = ""
        self._speaking = False
        self._using_fallback = False
        self._listening_by_chat = {}
        self.create_timer(0.1, self.on_timer)
        self.create_timer(2.0, self._refresh_operating_mode)

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
        self.create_subscription(Bool, "/voice_activity", self.on_voice_activity, 10)
        self.create_subscription(
            VoiceAssistantState,
            "voice_assistant_state",
            self.on_voice_assistant_state,
            10,
        )
        self.create_subscription(
            ChatIsListening, "chat_is_listening", self.on_chat_is_listening, 10
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
        self.debug_log(
            f"message ready for '{path.name}' in {(time.monotonic() - t0) * 1000:.1f} ms"
        )
        self.publisher.publish(msg)
        self.debug_log(f"ros publish called for '{path.name}'")

    def show_default_animation(self):
        msg = DisplayImage()
        msg.id.value = 2  # PIB_EYES_ANIMATED
        msg.format.value = 0  # ANIMATED_GIF
        msg.data = []

        self.publisher.publish(msg)
        self.default_is_active = True
        self.get_logger().info("Default PIB animated eyes shown")

    def on_timer(self):
        if self.auto_return_seconds <= 0:
            return

        if self._applied_phase not in (None, PHASE_IDLE):
            return

        if self.default_is_active or self._hardware_vad_active:
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

            if (
                max_line_width <= width - 2 * padding
                and total_height <= height - 2 * padding
            ):
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

    def _refresh_operating_mode(self) -> None:
        mode = cached_key_store_mode()
        if mode == self._operating_mode:
            return
        self._operating_mode = mode
        self._apply_visible_state()

    def on_voice_assistant_state(self, msg: VoiceAssistantState) -> None:
        """chat_is_listening and this message are what the three states come from."""
        self._voice_turned_on = bool(getattr(msg, "turned_on", False))
        self._voice_chat_id = getattr(msg, "chat_id", "") or ""
        self._personality_id = getattr(msg, "personality_id", "") or ""
        self._personality_name = getattr(msg, "personality_name", "") or ""
        self._speaking = bool(getattr(msg, "speaking", False))
        self._using_fallback = bool(getattr(msg, "using_fallback", False))
        self._apply_visible_state()

    def on_chat_is_listening(self, msg: ChatIsListening) -> None:
        chat_id = getattr(msg, "chat_id", "") or ""
        if not chat_id:
            return
        self._listening_by_chat[chat_id] = bool(getattr(msg, "listening", False))
        self._apply_visible_state()

    def _apply_visible_state(self) -> None:
        listening = self._listening_by_chat.get(self._voice_chat_id, False)
        state = resolve_visible_state(
            turned_on=self._voice_turned_on,
            chat_id=self._voice_chat_id,
            personality_id=self._personality_id,
            personality_name=self._personality_name,
            listening=listening,
            listening_chat_id=self._voice_chat_id if listening else "",
            speaking=self._speaking,
            using_fallback=self._using_fallback,
            operating_mode=self._operating_mode,
        )
        if (
            state.phase == self._applied_phase
            and state.display_text == self._applied_text
        ):
            return
        self._applied_phase = state.phase
        self._applied_text = state.display_text
        if state.phase == PHASE_IDLE:
            if self._hardware_vad_active:
                text = String()
                text.data = "listening"
                self.on_display_text(text)
                return
            self.show_default_animation()
            return
        self._show_phase(state.phase, state.display_text or state.phase)

    def _show_phase(self, phase: str, caption: str) -> None:
        """Animated face for the phase, with the holder named on the display.

        The drawing carries both: the eyes and mouth are the face, and the
        caption is who holds the voice. A missing expression file cannot drop
        the name.
        """
        self.last_expression_time = time.monotonic()
        self.default_is_active = False
        try:
            self.publish_png_bytes(self.render_phase_png(phase, caption))
            self.get_logger().info(f"Conversation face shown: {caption!r}")
        except Exception as exc:
            self.get_logger().error(f"Could not show conversation face: {exc}")

    def render_phase_png(self, phase: str, caption: str) -> bytes:
        """A face for listening, thinking, speaking, degraded or fallback."""
        width = int(os.environ.get("PIB_DISPLAY_WIDTH", "800"))
        height = int(os.environ.get("PIB_DISPLAY_HEIGHT", "480"))
        image = Image.new("RGBA", (width, height), (0, 0, 0, 255))
        draw = ImageDraw.Draw(image)
        eye = (69, 183, 255, 255)
        left = width // 2 - 140
        right = width // 2 + 60
        top = height // 2 - 80
        if phase == "degraded":
            draw.line((left, top + 30, left + 80, top + 70), fill=eye, width=10)
            draw.line((left + 80, top + 30, left, top + 70), fill=eye, width=10)
            draw.line((right, top + 30, right + 80, top + 70), fill=eye, width=10)
            draw.line((right + 80, top + 30, right, top + 70), fill=eye, width=10)
        else:
            draw.ellipse((left, top, left + 80, top + 80), outline=eye, width=8)
            draw.ellipse((right, top, right + 80, top + 80), outline=eye, width=8)
            pupil_dx = 18 if phase == "thinking" else 28
            draw.ellipse(
                (left + pupil_dx, top + 28, left + pupil_dx + 24, top + 52),
                fill=eye,
            )
            draw.ellipse(
                (right + pupil_dx, top + 28, right + pupil_dx + 24, top + 52),
                fill=eye,
            )
        mouth_y = height // 2 + 50
        mouth_x = width // 2
        if phase == "speaking":
            draw.ellipse(
                (mouth_x - 36, mouth_y, mouth_x + 36, mouth_y + 48),
                outline=eye,
                width=8,
            )
        elif phase == "listening":
            draw.arc(
                (mouth_x - 40, mouth_y, mouth_x + 40, mouth_y + 36),
                start=20,
                end=160,
                fill=eye,
                width=8,
            )
        else:
            draw.line(
                (mouth_x - 36, mouth_y + 16, mouth_x + 36, mouth_y + 16),
                fill=eye,
                width=8,
            )
        font = self.load_font(36)
        y = height - 120
        for line in str(caption).splitlines():
            box = draw.textbbox((0, 0), line, font=font)
            line_width = box[2] - box[0]
            draw.text(((width - line_width) // 2, y), line, font=font, fill=eye)
            y += 44
        buffer = io.BytesIO()
        image.save(buffer, format="PNG")
        return buffer.getvalue()

    def on_voice_activity(self, msg: Bool) -> None:
        """Show listening while the array hears speech, then return to the eyes."""
        active = bool(msg.data)
        if active == self._hardware_vad_active:
            return
        self._hardware_vad_active = active
        if self._applied_phase not in (None, PHASE_IDLE):
            return
        if active:
            text = String()
            text.data = "listening"
            self.on_display_text(text)
            return
        self.show_default_animation()
        self.default_is_active = True

    def on_display_text(self, msg: String):
        t0 = time.monotonic()
        raw = msg.data
        self.debug_log(f"display_text received chars={len(raw)} text='{raw[:40]}'")

        try:
            self.last_expression_time = time.monotonic()
            self.default_is_active = False

            t_render = time.monotonic()
            png = self.render_text_png(raw)
            self.debug_log(
                f"text rendered png_bytes={len(png)} in {(time.monotonic() - t_render) * 1000:.1f} ms"
            )

            t_publish = time.monotonic()
            self.publish_png_bytes(png)
            self.debug_log(
                f"text published in {(time.monotonic() - t_publish) * 1000:.1f} ms"
            )

            self.get_logger().info(
                f"Display text shown: {raw[:40]} total={(time.monotonic() - t0) * 1000:.1f} ms"
            )
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
            self.debug_log(
                f"file resolved expression='{expression}' file='{path}' in {(time.monotonic() - t_find) * 1000:.1f} ms"
            )

            t_pub = time.monotonic()
            self.publish_expression_file(path)
            self.debug_log(
                f"published expression='{expression}' in {(time.monotonic() - t_pub) * 1000:.1f} ms"
            )

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
