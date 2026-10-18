#!/usr/bin/env python3
"""Does the robot's own voice reach the microphone array's echo canceller? (PC-88)

The array is capture-only over USB, so the canceller can only subtract a playback
signal that arrives at its own analogue input. Whether that wiring exists is not
visible in software - it has to be measured, and this script does exactly that:
play a known sweep, record the same room, compare.

Two modes:

    python scripts/aec_probe.py record --seconds 3 --out /tmp/probe
        Plays a sweep through the default output while recording from the array.
        Writes /tmp/probe-reference.wav and /tmp/probe-capture.wav.

    python scripts/aec_probe.py compare /tmp/probe-reference.wav /tmp/probe-capture.wav
        Computes the echo return loss enhancement from two files. Use this when the
        recording was made with another tool.

Verdicts (see ``hid_tuning.erle_db``): above ~10 dB the reference is doing real
work; near 0 dB pib is hearing itself through the air and the barge-in
confirmation has to live with its own echo.
"""

from __future__ import annotations

import argparse
import json
import sys
import wave
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "ros_packages" / "voice_assistant"))

from voice_assistant.audio_devices import choose_input  # noqa: E402
from voice_assistant.hid_tuning import apply_tuning, erle_db, open_hid  # noqa: E402

RATE = 16_000
CHANNELS = 1


def sweep(
    seconds: float, rate: int = RATE, low: float = 200.0, high: float = 6000.0
) -> np.ndarray:
    """A linear sweep at a level loud enough to be cancelled but not to clip."""
    count = int(rate * seconds)
    t = np.arange(count) / rate
    frequency = low + (high - low) * t / max(t[-1], 1e-6)
    phase = 2 * np.pi * np.cumsum(frequency) / rate
    return (0.25 * 32767 * np.sin(phase)).astype(np.int16)


def write_wav(path: Path, samples: np.ndarray, rate: int = RATE) -> None:
    with wave.open(str(path), "wb") as handle:
        handle.setnchannels(CHANNELS)
        handle.setsampwidth(2)
        handle.setframerate(rate)
        handle.writeframes(np.asarray(samples, dtype="<i2").tobytes())


def read_wav(path: Path) -> np.ndarray:
    with wave.open(str(path), "rb") as handle:
        if handle.getsampwidth() != 2:
            raise SystemExit(f"{path}: expected 16-bit PCM")
        raw = handle.readframes(handle.getnframes())
        channels = handle.getnchannels()
    samples = np.frombuffer(raw, dtype="<i2")
    if channels > 1:
        samples = samples.reshape(-1, channels)[:, 0]
    return samples


def record(seconds: float, out: Path, *, tune: bool) -> int:
    import pyaudio  # imported here so --compare works without the hardware stack

    hid = open_hid() if tune else None
    if tune:
        outcome = apply_tuning(hid)
        print(
            json.dumps(
                {
                    "tuning": {
                        "applied": list(outcome.applied),
                        "refused": [list(item) for item in outcome.refused],
                    }
                }
            )
        )
    import pyaudio as _pa  # noqa: F401 - keeps the alias above honest

    audio = pyaudio.PyAudio()
    devices = [
        audio.get_device_info_by_index(i)["name"]
        for i in range(audio.get_device_count())
    ]
    choice = choose_input(devices)
    print(
        json.dumps(
            {"input": choice.name, "array": choice.is_array, "reason": choice.reason}
        )
    )

    reference = sweep(seconds)
    captured = np.zeros_like(reference)
    frame = 320

    output = audio.open(
        format=pyaudio.paInt16,
        channels=CHANNELS,
        rate=RATE,
        output=True,
        frames_per_buffer=frame,
    )
    capture = audio.open(
        format=pyaudio.paInt16,
        channels=CHANNELS,
        rate=RATE,
        input=True,
        frames_per_buffer=frame,
        input_device_index=_index_of(audio, choice.name),
    )
    try:
        for offset in range(0, len(reference), frame):
            chunk = reference[offset : offset + frame]
            output.write(chunk.tobytes())
            captured[offset : offset + frame] = np.frombuffer(
                capture.read(frame, exception_on_overflow=False), dtype="<i2"
            )[: len(chunk)]
    finally:
        output.close()
        capture.close()
        audio.terminate()

    write_wav(out.with_name(out.name + "-reference.wav"), reference)
    write_wav(out.with_name(out.name + "-capture.wav"), captured)
    db = erle_db(reference, captured)
    print(
        json.dumps(
            {
                "reference": str(out.with_name(out.name + "-reference.wav")),
                "capture": str(out.with_name(out.name + "-capture.wav")),
                "erle_db": round(db, 1),
                "verdict": (
                    "reference is being cancelled"
                    if db >= 10
                    else "little or no cancellation: check the array's input wiring"
                ),
            }
        )
    )
    return 0


def _index_of(audio, name: str) -> int | None:
    for index in range(audio.get_device_count()):
        info = audio.get_device_info_by_index(index)
        if info["name"] == name and info.get("maxInputChannels", 0) > 0:
            return index
    return None


def compare(reference_path: Path, capture_path: Path) -> int:
    reference = read_wav(reference_path)
    captured = read_wav(capture_path)
    db = erle_db(reference, captured)
    print(
        json.dumps(
            {
                "reference": str(reference_path),
                "capture": str(capture_path),
                "erle_db": round(db, 1),
                "verdict": (
                    "reference is being cancelled"
                    if db >= 10
                    else "little or no cancellation: check the array's input wiring"
                ),
            }
        )
    )
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    play = sub.add_parser("record", help="play a sweep and record the room")
    play.add_argument("--seconds", type=float, default=3.0)
    play.add_argument("--out", type=Path, default=Path("/tmp/aec-probe"))
    play.add_argument(
        "--tune", action="store_true", help="apply the HID switches first"
    )

    offline = sub.add_parser("compare", help="measure two wav files")
    offline.add_argument("reference", type=Path)
    offline.add_argument("capture", type=Path)

    args = parser.parse_args(argv)
    if args.command == "record":
        return record(args.seconds, args.out, tune=args.tune)
    return compare(args.reference, args.capture)


if __name__ == "__main__":
    raise SystemExit(main())
