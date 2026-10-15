"""Actual player methods with isolated engines/devices and deterministic clear races."""

from __future__ import annotations

import ast
import contextlib
import importlib.util
import io
import logging
import os
import sys
import time
import wave
from pathlib import Path
from queue import Empty, Queue
from threading import Event, Lock, Thread
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "ros_packages/voice_assistant/voice_assistant/audio_player.py"


@pytest.fixture
def player(monkeypatch):
    spec = importlib.util.spec_from_file_location(
        "_pc49_player_turn_taking",
        ROOT / "pib_hermes_config/pib_hermes_config/turn_taking.py",
    )
    assert spec is not None and spec.loader is not None
    helper = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, helper)
    spec.loader.exec_module(helper)
    ns = dict(
        __name__="_pc49_actual_player",
        Node=object,
        pyaudio=None,
        io=io,
        os=os,
        sys=sys,
        time=time,
        wave=wave,
        logging=logging,
        Queue=Queue,
        Empty=Empty,
        Lock=Lock,
        Event=Event,
        Thread=Thread,
        clauses_to_synthesize=helper.clauses_to_synthesize,
        resolve_tts_route=lambda engine: "local",
        util=SimpleNamespace(surpress_stderr=contextlib.nullcontext),
    )
    tree = ast.parse(SOURCE.read_text())
    body = [
        ast.ImportFrom(
            module="__future__", names=[ast.alias(name="annotations")], level=0
        )
    ]
    # No ROS/native imports, but all real classes, helper functions and constants.
    body += [
        n
        for n in tree.body
        if isinstance(n, (ast.ClassDef, ast.FunctionDef, ast.Assign, ast.AnnAssign))
    ]
    module = ast.Module(body=body, type_ignores=[])
    ast.fix_missing_locations(module)
    exec(compile(module, str(SOURCE), "exec"), ns)
    cls = ns["AudioPlayerNode"]
    node = cls.__new__(cls)
    node.playback_queue = Queue()
    node.counter = 0
    node.counter_lock = Lock()
    # Enforce the real rclpy one-message logger signature even at stub boundaries.
    logger = SimpleNamespace(
        **{
            method: MagicMock(side_effect=lambda message: None)
            for method in ("debug", "info", "warning", "error")
        }
    )
    node.get_logger = lambda: logger
    node._synthesize_clause = MagicMock(return_value=[b"\x01\x00"])
    ns["bind_playback_queue"](node.playback_queue)
    return node, SimpleNamespace(**ns)


def request(text="One sentence.", join=False):
    return SimpleNamespace(
        speech=text, gender="F1", language="de", tts_engine="supertone", join=join
    )


def test_clear_during_synthesis_rejects_whole_request_including_later_clauses(player):
    node, mod = player
    started, release = Event(), Event()
    calls = []

    def synth(text, req):
        calls.append(text)
        started.set()
        assert release.wait(2)
        return [b"\x01\x00"]

    node._synthesize_clause = synth
    worker = Thread(
        target=node.play_audio_from_speech,
        args=(request("First sentence. Second sentence.", True), SimpleNamespace()),
        daemon=True,
    )
    worker.start()
    assert started.wait(2)
    node.clear_playback_queue(None, SimpleNamespace())
    release.set()
    worker.join(2)
    assert not worker.is_alive()
    assert node.playback_queue.empty()
    assert len(calls) == 1  # suffix cannot receive a new post-clear generation


def test_clear_between_clauses_does_not_admit_a_fresh_suffix(player):
    node, _ = player
    enqueue = node._enqueue_current
    calls = []

    def clear_after_first(item):
        accepted = enqueue(item)
        calls.append(item)
        if len(calls) == 1:
            node.clear_playback_queue(None, SimpleNamespace())
        return accepted

    node._enqueue_current = clear_after_first
    node.play_audio_from_speech(
        request("First sentence. Second sentence."), SimpleNamespace()
    )
    assert node._synthesize_clause.call_count == 1
    assert node.playback_queue.empty()


def test_enqueue_itself_rejects_work_reserved_before_clear(player):
    node, mod = player
    old = mod.PlaybackItem([b"old"], mod.SPEECH_ENCODING, 0, node.counter_next())
    node.clear_playback_queue(None, SimpleNamespace())
    assert node._enqueue_current(old) is False
    assert old.finished_playing.is_set() and node.playback_queue.empty()


def test_fresh_request_after_clear_is_admitted(player):
    node, mod = player
    node.clear_playback_queue(None, SimpleNamespace())
    node.play_audio_from_speech(request(), SimpleNamespace())
    item = node.playback_queue.get_nowait()
    assert not item.is_cleared()
    assert list(item.data) == [b"\x01\x00"]
    item.play()  # no native device in this unit boundary
    assert item.finished_playing.is_set()


def test_clear_drains_queued_barriers_and_releases_join_waiter(player):
    node, mod = player
    item = mod.PlaybackItem([b""], mod.SPEECH_ENCODING, 0, node.counter_next())
    assert node._enqueue_current(item)
    done = Event()
    worker = Thread(
        target=lambda: (node._wait_playback_finished(item, "test"), done.set()),
        daemon=True,
    )
    worker.start()
    assert not done.is_set()
    node.clear_playback_queue(None, SimpleNamespace())
    assert done.wait(2)
    worker.join(2)
    assert node.playback_queue.empty() and item.finished_playing.is_set()


def test_join_never_fabricates_completion_when_native_item_is_stuck(player):
    node, mod = player
    item = mod.PlaybackItem([b"pcm"], mod.SPEECH_ENCODING, 0, node.counter_next())
    # Active item is outside the queue; clear must still release its service waiter.
    done = Event()
    node._wait_playback_finished.__globals__["JOIN_WAIT_TIMEOUT_SECONDS"] = 0.0
    worker = Thread(
        target=lambda: (node._wait_playback_finished(item, "native"), done.set()),
        daemon=True,
    )
    worker.start()
    try:
        # Observe a real bounded wait/report cycle without relying on a timing sleep.
        deadline = time.monotonic() + 2
        while not node.get_logger().warning.called and time.monotonic() < deadline:
            done.wait(0.01)
        assert node.get_logger().warning.called
        assert not done.is_set() and not item.finished_playing.is_set()
    finally:
        node.clear_playback_queue(None, SimpleNamespace())
        worker.join(2)
    assert done.is_set()


def test_synthesis_failure_does_not_leave_a_join_waiter(player):
    node, _ = player
    node._synthesize_clause.side_effect = RuntimeError("synthesis unavailable")
    node.play_audio_from_speech(request(join=True), SimpleNamespace())
    assert node.playback_queue.empty()


def test_late_lazy_chunk_is_fenced_after_clear(player):
    node, _ = player
    order = node.counter_next()

    def chunks():
        yield b"first"
        node.clear_playback_queue(None, SimpleNamespace())
        yield b"stale"

    assert list(node._fence_pcm_iterable(chunks(), order)) == [b"first"]


def test_actual_pcm_writer_stops_after_clear_between_chunks(player):
    node, mod = player
    writes = []

    def write(chunk):
        writes.append(chunk)
        node.clear_playback_queue(None, SimpleNamespace())

    stream = SimpleNamespace(write=write, stop_stream=lambda: None, close=lambda: None)
    pya = SimpleNamespace(
        open=lambda **kw: stream,
        get_format_from_width=lambda width: 16,
        terminate=lambda: None,
    )
    node._enqueue_current.__globals__["pyaudio"] = SimpleNamespace(PyAudio=lambda: pya)
    item = mod.PlaybackItem(
        [b"first", b"stale"], mod.SPEECH_ENCODING, 0, node.counter_next()
    )
    item.play()
    assert writes == [b"first"] and item.finished_playing.is_set()


def test_clear_and_fresh_admission_are_atomic(player):
    node, mod = player
    clearing, release_clear, fresh_attempted, fresh_admitted = (
        Event(),
        Event(),
        Event(),
        Event(),
    )

    class PausedDrainQueue(Queue):
        def get_nowait(self):
            try:
                return super().get_nowait()
            except Empty:
                clearing.set()
                assert release_clear.wait(2)
                raise

    node.playback_queue = PausedDrainQueue()
    old = mod.PlaybackItem([b"old"], mod.SPEECH_ENCODING, 0, node.counter_next())
    node._enqueue_current(old)
    clear = Thread(
        target=node.clear_playback_queue, args=(None, SimpleNamespace()), daemon=True
    )
    clear.start()
    assert clearing.wait(2)
    fresh = []

    def admit():
        fresh_attempted.set()
        item = mod.PlaybackItem([b"new"], mod.SPEECH_ENCODING, 0, node.counter_next())
        fresh.append(item)
        node._enqueue_current(item)
        fresh_admitted.set()

    new = Thread(target=admit, daemon=True)
    new.start()
    assert fresh_attempted.wait(2)
    assert not fresh_admitted.is_set()
    release_clear.set()
    clear.join(2)
    new.join(2)
    assert not clear.is_alive() and not new.is_alive()
    assert fresh_admitted.is_set()
    assert node.playback_queue.get_nowait() is fresh[0]
    assert old.finished_playing.is_set() and not fresh[0].is_cleared()


def test_fifo_clauses_and_empty_barrier_keep_order(player):
    node, mod = player
    node.play_audio_from_speech(
        request("First sentence. Second sentence."), SimpleNamespace()
    )
    node.play_audio_from_speech(request(""), SimpleNamespace())
    items = []
    while not node.playback_queue.empty():
        items.append(node.playback_queue.get_nowait())
    assert len(items) == 3
    assert items[0].order == items[1].order < items[2].order
    assert list(items[-1].data) == [b""]
    for item in items:
        item.play()
        assert item.finished_playing.is_set()


def test_clear_during_file_read_rejects_late_file(player, tmp_path):
    node, mod = player
    path = tmp_path / "tone.wav"
    with wave.open(str(path), "wb") as target:
        target.setnchannels(1)
        target.setsampwidth(2)
        target.setframerate(44100)
        target.writeframes(b"\x01\x00" * 20)
    started, release = Event(), Event()

    def gated_open(*args, **kwargs):
        result = wave.open(*args, **kwargs)
        started.set()
        assert release.wait(2)
        return result

    node.play_audio_from_file.__globals__["wave"] = SimpleNamespace(open=gated_open)
    worker = Thread(
        target=node.play_audio_from_file,
        args=(SimpleNamespace(filepath=str(path), join=True), SimpleNamespace()),
        daemon=True,
    )
    worker.start()
    assert started.wait(2)
    node.clear_playback_queue(None, SimpleNamespace())
    release.set()
    worker.join(2)
    assert not worker.is_alive() and node.playback_queue.empty()
