"""Unit tests for the 500 ms continuous pre-roll buffer in AudioRecorderNode (PR-2187)."""

from __future__ import annotations

from collections import deque
from pathlib import Path
from threading import Event, Lock
from types import SimpleNamespace

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
AUDIO_RECORDER_SRC = (
    REPO_ROOT
    / "ros_packages"
    / "voice_assistant"
    / "voice_assistant"
    / "audio_recorder.py"
)


def test_audio_recorder_maintains_preroll_when_idle_and_drains_on_barge_in():
    source = AUDIO_RECORDER_SRC.read_text(encoding="utf-8")
    assert "PREROLL_DURATION_SECONDS = 0.5" in source
    assert "BARGE_IN_SILENT_BEFORE_SENTINEL = 0.05" in source
    assert "self.preroll_chunks = deque(maxlen=8)" in source
    assert "if is_barge_in and preroll_ring:" in source
    assert "chunks = list(preroll_ring)" in source


def test_preroll_ring_caps_at_eight_chunks_and_switches_to_active_queue():
    preroll = deque(maxlen=8)
    active = deque()
    lock = Lock()
    event = Event()
    recording_active = False

    def feed(sample_val: int):
        chunk = np.full(1024, sample_val, dtype=np.int16).tobytes()
        with lock:
            if recording_active:
                active.append(chunk)
                event.set()
            else:
                preroll.append(chunk)

    # Feed 12 chunks while idle -> only latest 8 (500 ms @ 16 kHz / 1024 frames) remain
    for i in range(12):
        feed(i)
    assert len(preroll) == 8
    first_kept = np.frombuffer(preroll[0], dtype=np.int16)[0]
    assert first_kept == 4

    # Start barge-in recording -> drains 8 pre-roll chunks into chunks and switches to active
    with lock:
        chunks = list(preroll)
        preroll.clear()
        recording_active = True

    assert len(chunks) == 8
    assert len(preroll) == 0

    feed(99)
    assert len(active) == 1
    assert len(preroll) == 0
