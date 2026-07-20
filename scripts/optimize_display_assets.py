#!/usr/bin/env python3
"""Resize display assets to a fixed resolution for faster runtime loading."""

from __future__ import annotations

import argparse
import hashlib
import logging
import sys
import time
from pathlib import Path

from PIL import Image

IMAGE_SUFFIXES = {".png", ".jpg", ".jpeg", ".gif"}


def file_hash(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def resize_image(image: Image.Image, width: int, height: int) -> Image.Image:
    if image.size == (width, height):
        return image
    return image.resize((width, height), Image.LANCZOS)


def optimize_static(path: Path, width: int, height: int) -> bytes | None:
    suffix = path.suffix.lower()
    with Image.open(path) as image:
        resized = resize_image(image, width, height)
        if image.size == (width, height) and suffix in (".png", ".jpg", ".jpeg"):
            return None

        buffer = __import__("io").BytesIO()
        if suffix in (".jpg", ".jpeg"):
            rgb = resized.convert("RGB")
            rgb.save(buffer, format="JPEG", quality=90, optimize=True)
        else:
            resized.save(buffer, format="PNG", optimize=True)
        return buffer.getvalue()


def optimize_gif(path: Path, width: int, height: int) -> bytes | None:
    with Image.open(path) as image:
        frames: list[Image.Image] = []
        durations: list[int] = []

        for index in range(image.n_frames):
            image.seek(index)
            frame = image.copy()
            durations.append(int(image.info.get("duration", 80)))
            frames.append(resize_image(frame, width, height))

        if image.size == (width, height) and len(frames) == image.n_frames:
            return None

        buffer = __import__("io").BytesIO()
        frames[0].save(
            buffer,
            format="GIF",
            save_all=True,
            append_images=frames[1:],
            duration=durations,
            loop=image.info.get("loop", 0),
            optimize=True,
        )
        return buffer.getvalue()


def process_file(path: Path, width: int, height: int, in_place: bool, output_dir: Path | None) -> bool:
    start = time.monotonic()
    original_size = path.stat().st_size

    if path.suffix.lower() == ".gif":
        optimized = optimize_gif(path, width, height)
    elif path.suffix.lower() in IMAGE_SUFFIXES:
        optimized = optimize_static(path, width, height)
    else:
        return False

    if optimized is None:
        logging.info("skip %s (already %dx%d)", path.name, width, height)
        return False

    target = (output_dir / path.name) if output_dir else path
    if output_dir:
        target.parent.mkdir(parents=True, exist_ok=True)

    before_hash = file_hash(path)
    target.write_bytes(optimized)
    after_hash = file_hash(target)
    elapsed_ms = (time.monotonic() - start) * 1000

    if in_place and target == path and before_hash == after_hash:
        logging.info("skip %s (unchanged)", path.name)
        return False

    logging.info(
        "optimized %s -> %dx%d (%d -> %d bytes, %.1f ms)",
        path.name,
        width,
        height,
        original_size,
        target.stat().st_size,
        elapsed_ms,
    )
    return True


def iter_images(directory: Path) -> list[Path]:
    if not directory.exists():
        logging.warning("directory missing: %s", directory)
        return []
    return sorted(
        path
        for path in directory.iterdir()
        if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Optimize display assets to a fixed resolution.")
    parser.add_argument("--width", type=int, default=1024)
    parser.add_argument("--height", type=int, default=600)
    parser.add_argument("--expressions-dir", type=Path, required=True)
    parser.add_argument("--static-dir", type=Path, required=True)
    parser.add_argument("--in-place", action="store_true")
    parser.add_argument("--output-dir", type=Path, default=None)
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(message)s")

    if args.in_place and args.output_dir:
        logging.error("Use either --in-place or --output-dir, not both.")
        return 1

    changed = 0
    for directory in (args.expressions_dir, args.static_dir):
        for path in iter_images(directory):
            if process_file(path, args.width, args.height, args.in_place, args.output_dir):
                changed += 1

    logging.info("done, updated %d file(s)", changed)
    return 0


if __name__ == "__main__":
    sys.exit(main())
