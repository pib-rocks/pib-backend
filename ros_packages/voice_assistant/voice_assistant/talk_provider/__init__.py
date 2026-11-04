"""Hermes Talk's Gemini Live provider, vendored into the voice assistant.

Source: https://github.com/TheSmokeDev/hermes-talk (MIT), provider commit
7ad833e4fcedb2c9a9947d84d7fd89bdbd15d256, files ``talk_realtime.py`` and
``talk_gemini_realtime.py`` unchanged except for this package marker.

Why vendored: the live engine must run inside the ROS container without a Hermes
installation. The modules are self-contained; ``aiohttp`` is imported lazily at
connect time, so importing the package needs nothing beyond the stdlib.
"""

from . import talk_gemini_realtime, talk_realtime

__all__ = ["talk_gemini_realtime", "talk_realtime"]
