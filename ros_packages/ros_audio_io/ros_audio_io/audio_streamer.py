# audio_streamer.py

import json
import os

import numpy as np
import pyaudio
import rclpy
from datatypes.srv import GetMicConfiguration
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int16MultiArray, String

from ros_audio_io.device_retry import (
    DeviceNotFoundError,
    describe_open_failure,
    device_status_payload,
    next_retry_delay,
)
from ros_audio_io.levels import calculate_levels


class AudioStreamer(Node):
    """
    Capture from a multichannel USB mic (e.g., ReSpeaker v3.x) and publish ONLY one selected channel
    as mono Int16 PCM chunks on the 'audio_stream' topic.

    Env vars:
      MIC_DEVICE              substring to match input device name (default: "default")
      MIC_CHANNELS            number of input channels to open (default: 6)
      MIC_RATE                sample rate to request (default: 16000)
      MIC_PROCESSED_CHANNEL   which input channel index to publish (default: 0)

    Notes:
      - For ReSpeaker v3.x with 6-ch firmware: wiki ch0 is the AEC/beamformed stream.
      - If opening with MIC_CHANNELS fails, we fallback to the device's maxInputChannels.
      - If the requested sample rate is not supported, we fallback to the device's defaultSampleRate.
    """

    def __init__(self):
        super().__init__("audio_streamer")

        # --- Config from env ---
        self.mic_preferred_name = os.getenv("MIC_DEVICE", "default").lower()
        self.requested_channels = int(os.getenv("MIC_CHANNELS", "6"))
        self.requested_rate = int(os.getenv("MIC_RATE", "16000"))
        self.processed_channel_index = int(os.getenv("MIC_PROCESSED_CHANNEL", "0"))

        # --- Audio params ---
        self.chunk_size = 1024
        self.audio_format = pyaudio.paInt16  # 16-bit
        self.input_device_index = None
        self.sample_rate = None
        self.open_channels = None
        self.audio_stream = None
        self.timer = None
        self.retry_timer = None
        self.open_attempts = 0
        self.py_audio = None

        # --- ROS pub/service ---
        self.pub = self.create_publisher(Int16MultiArray, "audio_stream", 10)
        self.levels_pub = self.create_publisher(
            Float32MultiArray, "/microphone_levels", 10
        )
        self.status_pub = self.create_publisher(String, "/microphone_device_status", 10)
        self.srv = self.create_service(
            GetMicConfiguration, "get_mic_configuration", self.get_mic_configuration
        )

        # --- PyAudio init & device selection ---
        self._attempt_open_device()
        self.get_logger().info(
            "Mic configuration service ready (get_mic_configuration)"
        )

    def _open_device(self):
        if self.py_audio is None:
            self.py_audio = pyaudio.PyAudio()
        self.input_device_index = None
        self.select_input_device()
        dev = self.py_audio.get_device_info_by_index(self.input_device_index)
        dev_name = dev.get("name")
        dev_default_rate = int(dev.get("defaultSampleRate", 16000))
        dev_max_in = int(dev.get("maxInputChannels", 0))

        # Choose rate/channels with graceful fallback
        sr_candidates = [self.requested_rate, dev_default_rate]
        ch_candidates = [self.requested_channels, dev_max_in]

        last_err = None

        for rate in sr_candidates:
            for channel in ch_candidates:
                if channel <= 0:
                    continue

                self.get_logger().info(
                    f"Trying to open '{dev_name}' (idx {self.input_device_index}) "
                    f"rate={rate}Hz channels={channel} format=Int16 chunk={self.chunk_size}"
                )

                try:
                    stream = self.py_audio.open(
                        format=self.audio_format,
                        channels=channel,
                        rate=rate,
                        input=True,
                        input_device_index=self.input_device_index,
                        frames_per_buffer=self.chunk_size,
                    )
                except Exception as e:
                    last_err = e
                    continue  # try next combination

                # success: store and exit both loops
                self.audio_stream = stream
                self.sample_rate = rate
                self.open_channels = channel
                break  # exit inner loop
            else:
                # only executed if inner loop didn't break → try next rate
                continue
            break  # exit outer loop once success found
        else:
            raise last_err or RuntimeError("no valid rate/channel combination")

        # Clamp selected channel to valid range
        if self.open_channels <= 0:
            raise RuntimeError("opened device reported no input channels")

        if (
            self.processed_channel_index < 0
            or self.processed_channel_index >= self.open_channels
        ):
            self.get_logger().warning(
                f"MIC_PROCESSED_CHANNEL={self.processed_channel_index} out of range "
                f"(0..{self.open_channels-1}); using 0."
            )
            self.processed_channel_index = 0

        return dev_name

    def _attempt_open_device(self):
        self.open_attempts += 1
        try:
            dev_name = self._open_device()
        except Exception as exc:
            self.audio_stream = None
            self.sample_rate = None
            self.open_channels = None
            if self.py_audio is not None:
                self.py_audio.terminate()
                self.py_audio = None
            delay = next_retry_delay(self.open_attempts)
            reason = describe_open_failure(exc)
            detail = str(exc)
            self.get_logger().warning(
                f"Microphone open failed: reason={reason}; detail={detail}; "
                f"attempt={self.open_attempts}; next attempt in {delay:g}s"
            )
            self._publish_status(
                device_status_payload(
                    available=False,
                    reason=reason,
                    detail=detail,
                    attempts=self.open_attempts,
                    next_retry_in_seconds=delay,
                )
            )
            self.retry_timer = self.create_timer(delay, self.retry_open_device)
            return

        self.get_logger().info(
            "=== Input configured ===\n"
            f"Device:      {dev_name} (index {self.input_device_index})\n"
            f"Rate:        {self.sample_rate} Hz\n"
            f"Channels:    {self.open_channels} (publishing ONLY channel {self.processed_channel_index})\n"
            f"Chunk size:  {self.chunk_size} frames\n"
            f"Format:      Int16\n"
            f"Env: MIC_DEVICE='{self.mic_preferred_name}', MIC_CHANNELS={self.requested_channels}, "
            f"MIC_RATE={self.requested_rate}, MIC_PROCESSED_CHANNEL={self.processed_channel_index}"
        )
        self._publish_status(
            device_status_payload(
                available=True,
                device_name=dev_name,
                channels=self.open_channels,
                rate=self.sample_rate,
                processed_channel=self.processed_channel_index,
            )
        )

        # Publish timer
        self.timer = self.create_timer(
            self.chunk_size / float(self.sample_rate), self.publish_audio
        )

    def retry_open_device(self):
        retry_timer = self.retry_timer
        self.retry_timer = None
        if retry_timer is not None:
            retry_timer.cancel()
            self.destroy_timer(retry_timer)
        self._attempt_open_device()

    def _publish_status(self, payload):
        message = String()
        message.data = json.dumps(payload)
        self.status_pub.publish(message)

    # ----- ROS service -----
    def get_mic_configuration(self, request, response):
        response.mic_channels = 1  # we publish mono (selected channel only)
        response.chunk_size = self.chunk_size
        response.audio_format = self.audio_format
        response.sample_rate = self.sample_rate or 0
        return response

    # ----- Helpers -----
    def select_input_device(self):
        """Select preferred mic by substring; else fall back to default."""
        found = None
        hostapis = {
            self.py_audio.get_host_api_info_by_index(i)[
                "index"
            ]: self.py_audio.get_host_api_info_by_index(i)["name"]
            for i in range(self.py_audio.get_host_api_count())
        }
        for i in range(self.py_audio.get_device_count()):
            info = self.py_audio.get_device_info_by_index(i)
            name = info.get("name", "")
            max_in = info.get("maxInputChannels", 0)
            api_name = hostapis.get(info.get("hostApi"), "?")
            self.get_logger().info(
                f"Device {i}: '{name}' | hostapi={api_name} | maxInputChannels={max_in}"
            )
            if max_in and self.mic_preferred_name in name.lower():
                found = i
                break

        if found is not None:
            self.input_device_index = found
            sel = self.py_audio.get_device_info_by_index(found)
            self.get_logger().info(
                f"Using preferred input '{sel.get('name')}' (index {found})"
            )
        else:
            try:
                default_info = self.py_audio.get_default_input_device_info()
                self.input_device_index = int(default_info.get("index"))
                self.get_logger().warning(
                    f"No device matching '{self.mic_preferred_name}'; "
                    f"falling back to default input '{default_info.get('name')}' (index {self.input_device_index})"
                )
            except (IOError, OSError) as exc:
                raise DeviceNotFoundError(
                    f"No device matching '{self.mic_preferred_name}' and no "
                    f"default input: {exc}"
                )

    # ----- Main loop -----
    def publish_audio(self):
        try:
            raw = self.audio_stream.read(self.chunk_size, exception_on_overflow=False)
            buf = np.frombuffer(raw, dtype=np.int16)

            # De-interleave if multichannel, pick just the processed channel
            if self.open_channels > 1:
                frames = buf.size // self.open_channels
                if frames * self.open_channels != buf.size:
                    # Truncate to whole frames if we got a partial
                    buf = buf[: frames * self.open_channels]
                buf = buf.reshape(frames, self.open_channels)[
                    :, self.processed_channel_index
                ]

            # Publish mono PCM (selected channel only)
            msg = Int16MultiArray()
            msg.data = buf.tolist()
            self.pub.publish(msg)

            # Float32MultiArray layout: [normalized RMS, normalized peak].
            # The published PCM is mono; raw microphone channels are not exposed.
            rms, peak = calculate_levels(buf)
            levels_msg = Float32MultiArray()
            levels_msg.data = [rms, peak]
            self.levels_pub.publish(levels_msg)

        except Exception as e:
            self.get_logger().error(f"Audio read/publish error: {e}")
            # Optional: attempt a soft recover on I/O errors. For now, just continue.

    def destroy_node(self):
        try:
            if self.audio_stream is not None:
                self.audio_stream.stop_stream()
                self.audio_stream.close()
        finally:
            if self.py_audio is not None:
                self.py_audio.terminate()
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AudioStreamer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
