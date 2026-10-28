# ReSpeaker mic array and PipeWire (PR-2087)

On a pib with a **Seeed ReSpeaker Mic Array v2.0** (USB UAC 1.0, XVF-3000), the
host runs **PipeWire** with **WirePlumber**. Only `ros_audio_io/audio_streamer.py`
opens the capture device and publishes mono Int16 on `audio_stream`; this runbook
is about the **host** graph so that node sees six channels and channel 0 stays the
processed one.

## Why channel 0 matters

In **6-channel firmware**, the array exposes **S16_LE, 16000 Hz, 6 channels**:

| Channel | Content |
| --- | --- |
| **0** | Beamformed, echo-cancelled stream from the XVF-3000 (what the voice stack should use) |
| 1–5 | Raw microphone capsules |

`audio_streamer` selects channel 0 and publishes that alone. The selection logic
and name hints live in `voice_assistant/audio_devices.py` (`respeaker`, `seeed`,
`uac1.0`).

## What goes wrong when PipeWire downmixes

If WirePlumber exposes the mic as **stereo** or **downmixes** in the graph, channel
0 is no longer isolated: energy from the raw microphones is mixed in. Playback from
the robot's speaker then leaks into the stream the transcriber hears — **pib appears
to hear itself** even when wiring and AEC are correct.

Setup installs `51-pib-respeaker-mic-array.conf` (same hints as above) so the ALSA
source stays at **six channels**, **16 kHz**, with **channelmix disabled**, and the
ReSpeaker is preferred over **HDMI monitor** sources as the default capture device.

## Verify on the robot (as user `pib`)

Check defaults and channel count:

```bash
wpctl status
wpctl inspect @DEFAULT_AUDIO_SOURCE@
pactl list sources | sed -n '/Name:/,/Channel Map:/p'
```

You want the default source description to match the ReSpeaker (name contains
`respeaker`, `seeed`, or `uac1.0`), and **Channels: 6** (not 1 or 2).

Short capture to confirm six channels (adjust `source` if `pactl list sources short`
shows a different name):

```bash
parec --device=@DEFAULT_SOURCE@ --format=s16le --rate=16000 --channels=6 \
  /tmp/respeaker-6ch.raw &
PID=$!
sleep 2
kill $PID
wc -c /tmp/respeaker-6ch.raw
# Expect about 2 s * 16000 Hz * 6 ch * 2 bytes ≈ 384000 bytes
```

If an old desktop session saved HDMI as the default source, clear it and let
priorities apply again:

```bash
wpctl clear-default
systemctl --user restart wireplumber
```

Drop-in path after setup: `~/.config/wireplumber/wireplumber.conf.d/51-pib-respeaker-mic-array.conf`.
