# audio_streamer.py


import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from datatypes.srv import GetMicConfiguration
import pyaudio
import numpy as np
import os
import traceback


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

        # --- ROS pub/service ---
        self.pub = self.create_publisher(Int16MultiArray, "audio_stream", 10)
        self.srv = self.create_service(
            GetMicConfiguration, "get_mic_configuration", self.get_mic_configuration
        )

        # --- PyAudio init & device selection ---
        self.py_audio = pyaudio.PyAudio()
        self.select_input_device()
        if self.input_device_index is None:
            self.get_logger().error("No audio input device found; shutting down.")
            rclpy.shutdown()
            return

        dev = self.py_audio.get_device_info_by_index(self.input_device_index)
        dev_name = dev.get("name")
        dev_default_rate = int(dev.get("defaultSampleRate", 16000))
        dev_max_in = int(dev.get("maxInputChannels", 0))

        # Choose rate/channels with graceful fallback
        sr_candidates = [self.requested_rate, dev_default_rate]
        ch_candidates = [self.requested_channels, dev_max_in]

        self.audio_stream = None
        last_err = None

        for rate in sr_candidates:
            for ch in ch_candidates:
                if ch <= 0:
                    continue
                try:
                    self.get_logger().info(
                        f"Trying to open '{dev_name}' (idx {self.input_device_index}) "
                        f"rate={rate}Hz channels={ch} format=Int16 chunk={self.chunk_size}"
                    )
                    stream = self.py_audio.open(
                        format=self.audio_format,
                        channels=ch,
                        rate=rate,
                        input=True,
                        input_device_index=self.input_device_index,
                        frames_per_buffer=self.chunk_size,
                    )
                    self.audio_stream = stream
                    self.sample_rate = rate
                    self.open_channels = ch
                    break
                except Exception as e:
                    last_err = e
                    self.get_logger().warning(
                        f"Open failed with rate={rate} ch={ch}: {e}"
                    )
            if self.audio_stream:
                break

        if not self.audio_stream:
            self.get_logger().error(
                "Failed to open audio stream after trying fallbacks.\n"
                + (
                    ""
                    if not last_err
                    else "".join(
                        traceback.format_exception_only(type(last_err), last_err)
                    ).strip()
                )
            )
            rclpy.shutdown()
            return

        # Clamp selected channel to valid range
        if self.open_channels <= 0:
            self.get_logger().error("Open channels invalid; shutting down.")
            rclpy.shutdown()
            return

        if (
            self.processed_channel_index < 0
            or self.processed_channel_index >= self.open_channels
        ):
            self.get_logger().warning(
                f"MIC_PROCESSED_CHANNEL={self.processed_channel_index} out of range "
                f"(0..{self.open_channels-1}); using 0."
            )
            self.processed_channel_index = 0

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
        self.get_logger().info(
            "Mic configuration service ready (get_mic_configuration)"
        )

        # Publish timer
        self.timer = self.create_timer(
            self.chunk_size / float(self.sample_rate), self.publish_audio
        )

    # ----- ROS service -----
    def get_mic_configuration(self, request, response):
        response.mic_channels = 1  # we publish mono (selected channel only)
        response.chunk_size = self.chunk_size
        response.audio_format = self.audio_format
        response.sample_rate = self.sample_rate
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
            except IOError:
                self.get_logger().error(
                    f"No device matching '{self.mic_preferred_name}' and no default input; shutting down."
                )
                self.input_device_index = None

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

        except Exception as e:
            self.get_logger().error(f"Audio read/publish error: {e}")
            # Optional: attempt a soft recover on I/O errors. For now, just continue.

    def destroy_node(self):
        try:
            if self.audio_stream is not None:
                self.audio_stream.stop_stream()
                self.audio_stream.close()
        finally:
            self.py_audio.terminate()
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AudioStreamer()
    if node.input_device_index is not None:
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
