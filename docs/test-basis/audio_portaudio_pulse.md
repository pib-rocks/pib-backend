# Audio images: PortAudio Pulse host API (PR-2085)

The `ros-voice-assistant` and `ros-audio-io` images install PyAudio against a PortAudio
library that exposes the **PulseAudio** host API (in addition to ALSA). Ubuntu’s packaged
`libportaudio` does not include that backend; the Dockerfiles rebuild PortAudio from
upstream with `libpulse-dev` and `-DPA_USE_PULSEAUDIO=ON` before `pip install pyaudio`.

At runtime, set `PULSE_SERVER` (as `ros-audio-io` already does on the robot) so capture
and playback go through PipeWire/Pulse instead of contending for raw ALSA `hw:` devices.
When `PULSE_SERVER` is unset, PortAudio skips an unreachable Pulse server and ALSA remains
available as a fallback (the Docker build applies a small upstream patch for that).

Microphone capture stays single-owner: only `ros_audio_io/audio_streamer.py` opens an
input stream and publishes `audio_stream`.
