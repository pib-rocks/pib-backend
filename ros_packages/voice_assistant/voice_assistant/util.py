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
