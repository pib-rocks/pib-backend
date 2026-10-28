# Audio images: reaching PipeWire from the containers (PR-2085)

Both audio images (`ros-voice-assistant`, `ros-audio-io`) install **`libasound2-plugins`**,
which provides ALSA's Pulse plugin (`libasound_module_pcm_pulse.so`). That plugin is what
lets PortAudio - whose ALSA backend the images already use - reach the host PipeWire.

## Why not build PortAudio with a Pulse host API

- Ubuntu's packaged `libportaudio` has no Pulse backend, which is what the first attempt
  tried to fix by rebuilding PortAudio from source.
- Upstream **v19.7.0 has no `src/hostapi/pulseaudio` at all**, so `-DPA_USE_PULSEAUDIO=ON`
  is a no-op there: the image would have looked built but exposed no Pulse API.
- Only unreleased upstream master carries that host API. Pinning master trades a
  reproducibility problem for an unmaintained fork, and the arm64 build of that approach
  failed on the device (`pulseaudio --daemonize` cannot run in the build container).

The ALSA plugin needs no source build, no upstream patch and no daemon: it is the
distribution-supported route.

## How the stack selects it

- `PULSE_SERVER` must point at the host socket (PR-2086 mounts it into the voice container;
  `ros-audio-io` already did).
- `pulse_plugin_device_index()` (audio-io) and `_pulse_plugin_device_index()` (voice) return
  the device named `pulse` when a server is configured. Capture and playback then open that
  device, so PipeWire owns the array and rate conversion (16 kHz mono against a 44.1/48 kHz
  HDMI sink) happens in the graph instead of failing with `-9994`.
- Without `PULSE_SERVER` the selection is unchanged: the raw ALSA device, with the readable
  failure reporting and capped backoff from PR-2088.

## Checking an image

```
ls /usr/lib/*/alsa-lib/libasound_module_pcm_pulse.so   # build-time guard asserts this
python3 -c "import pyaudio; p=pyaudio.PyAudio(); print([p.get_device_info_by_index(i)['name'] for i in range(p.get_device_count())])"
```

The second command lists `pulse` once the plugin is installed and `PULSE_SERVER` is set.

Microphone capture stays single-owner: `ros_audio_io/audio_streamer.py` is the only capture
owner and publishes `audio_stream`; every other consumer subscribes.
