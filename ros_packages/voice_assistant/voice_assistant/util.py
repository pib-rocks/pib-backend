import contextlib
import os
import sys


@contextlib.contextmanager
def surpress_stderr():
    """
    Surpresses stderr output to prevent PyAudio from spamming Alsa messages
    """
    devnull = os.open(os.devnull, os.O_WRONLY)
    old_stderr = os.dup(2)
    sys.stderr.flush()
    os.dup2(devnull, 2)
    os.close(devnull)
    try:
        yield
    finally:
        os.dup2(old_stderr, 2)
        os.close(old_stderr)


def speech_turn_id(chat_id: str, epoch: int) -> str:
    """Opaque id of one spoken answer.

    The audio player remembers the ids it was told to cancel, so a clause
    request that was dispatched before the interruption and only reaches the
    player afterwards is recognised as part of the cancelled answer instead of
    starting a second answer on top of the new one.
    """
    return f"{(chat_id or '').strip()}#{int(epoch)}"
