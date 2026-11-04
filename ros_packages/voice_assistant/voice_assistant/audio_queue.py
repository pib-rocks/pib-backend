"""Handing PCM from a ROS callback thread to the asyncio loop without waiting.

Kept free of rclpy so the behaviour is testable on a host without ROS.
"""

from __future__ import annotations

import asyncio


def enqueue_pcm(loop, queue, stop_evt, payload: bytes) -> None:
    """Queue one PCM buffer from the ROS thread without waiting for the loop.

    A blocking put here dropped whole syllables whenever the loop was busy with a
    transcript or a playback write, and the live model then answered half a
    sentence (heard on 192.168.1.168). The queue is unbounded, so putting never
    needs to wait for the loop.
    """
    if stop_evt.is_set():
        return

    async def _put():
        queue.put_nowait({"data": payload, "mime_type": "audio/pcm"})

    try:
        asyncio.run_coroutine_threadsafe(_put(), loop)
    except Exception:
        # A closed loop during shutdown; nothing to deliver to.
        pass
