"""Which microphone the voice stack listens to, and which channel of it (PC-88).

The Seeed ReSpeaker Mic Array v2.0 is a UAC 1.0 capture device with the XVF-3000
doing beamforming and echo cancellation in hardware. Two facts about it decide this
module:

* It exposes 1, 2, 4 or 6 channels depending on its firmware. In the 6-channel mode
  channel 0 is the processed one - echo cancelled and beamformed - and the other
  channels are the raw microphones. Feeding raw channels into the relay hands the
  transcriber pib's own voice.
* It is capture-only over USB. There is no output endpoint, so playback cannot be
  pointed at "the same device" and the echo canceller's reference has to reach the
  array's own analogue input. That is a wiring question, not a software switch,
  and :mod:`scripts.aec_probe` exists to answer it with a measurement.

The functions here are pure: they take device descriptions and interleaved sample
arrays, which is what makes the selection logic testable without the hardware.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

RESPEAKER_HINTS = ("respeaker", "seeed", "uac1.0", "xvf3000", "xvf-3000")
PROCESSED_CHANNEL = 0
"""The channel the array has already echo-cancelled and beamformed."""


@dataclass(frozen=True)
class DeviceChoice:
    """The input this session records from, and why."""

    name: str
    channels: int
    channel_index: int
    is_array: bool
    reason: str

    @property
    def processed(self) -> bool:
        return self.is_array


def choose_input(
    device_names: list[str], *, default: str | None = None
) -> DeviceChoice:
    """Pick the array if it is present, otherwise the default input.

    Names are the human labels ALSA/PortAudio report; matching on them is enough
    here because the caller only needs to know "array or not" and how many channels
    to expect. The channel count is read from the stream at open time, not guessed
    from the name - :func:`extract_channel` does the validation.
    """
    for name in device_names:
        lowered = name.lower()
        if any(hint in lowered for hint in RESPEAKER_HINTS):
            return DeviceChoice(
                name=name,
                channels=0,  # filled from the stream at open time
                channel_index=PROCESSED_CHANNEL,
                is_array=True,
                reason="microphone array found by name",
            )
    fallback = default or (device_names[0] if device_names else "")
    return DeviceChoice(
        name=fallback,
        channels=0,
        channel_index=0,
        is_array=False,
        reason="no microphone array in the device list; using the default input",
    )


def extract_channel(
    frames: np.ndarray, input_channels: int, channel: int = PROCESSED_CHANNEL
) -> np.ndarray:
    """Take one channel out of an interleaved capture buffer.

    Returns mono int16 samples ready for the relay. A single-channel stream is
    returned unchanged - the array in its 1-channel firmware already delivers the
    processed signal.
    """
    if input_channels < 1:
        raise ValueError("input_channels must be >= 1")
    if channel < 0 or channel >= input_channels:
        raise ValueError(
            f"channel {channel} does not exist in {input_channels} channels"
        )
    samples = np.asarray(frames)
    if samples.ndim == 1:
        if input_channels == 1:
            return samples.astype("<i2", copy=False)
        if samples.size % input_channels:
            raise ValueError("sample count is not a multiple of the channel count")
        return samples.reshape(-1, input_channels)[:, channel].astype("<i2", copy=False)
    if samples.ndim == 2:
        if samples.shape[1] != input_channels:
            raise ValueError("frame width does not match the channel count")
        return samples[:, channel].astype("<i2", copy=False)
    raise ValueError("expected a flat or two-dimensional sample array")
