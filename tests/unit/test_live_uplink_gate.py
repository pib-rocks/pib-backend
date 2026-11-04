"""Live uplink: no dropped syllables while the loop is busy, no echo while pib speaks."""

import asyncio
import threading
import time
from pathlib import Path

from voice_assistant.audio_queue import enqueue_pcm

REPO_ROOT = Path(__file__).resolve().parents[2]
AUDIO_LOOP = (
    REPO_ROOT / "ros_packages" / "voice_assistant" / "voice_assistant" / "audio_loop.py"
)


def test_a_busy_loop_still_receives_the_chunk_it_was_handed():
    """The ROS callback must not wait for the loop (it used to wait 250 ms)."""

    async def exercise():
        loop = asyncio.get_running_loop()
        queue: asyncio.Queue = asyncio.Queue()
        stop = threading.Event()

        loop.call_soon_threadsafe(time.sleep, 0.5)
        took = time.monotonic()
        enqueue_pcm(loop, queue, stop, b"\x00\x01")
        elapsed = time.monotonic() - took

        await asyncio.sleep(0.7)
        assert elapsed < 0.05, f"enqueue blocked for {elapsed:.3f}s"
        assert queue.qsize() == 1
        assert queue.get_nowait() == {"data": b"\x00\x01", "mime_type": "audio/pcm"}

    asyncio.run(exercise())


def test_the_uplink_is_closed_while_the_robot_speaks():
    """The echo gate sits in the send loop, before the playback-tail admission."""

    source = AUDIO_LOOP.read_text(encoding="utf-8")
    send_loop = source.split("async def send_realtime", 1)[1]
    body = send_loop.split("async with self._send_lock", 1)[0]
    assert "if self._speaking:" in body, "the echo gate is gone"
    assert body.index("if self._speaking:") < body.index("admit_live_uplink(")


def test_the_bridge_hands_chunks_over_without_a_timeout():
    source = AUDIO_LOOP.read_text(encoding="utf-8")
    assert "result(timeout=" not in source
    assert "enqueue_pcm(" in source
