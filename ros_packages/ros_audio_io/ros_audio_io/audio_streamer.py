import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
import pyaudio
import numpy as np
import os


class AudioStreamer(Node):
    def __init__(self):
        super().__init__("audio_streamer")

        # Read preferred device substring from environment (MIC_DEVICE)
        self.mic_preferred_name = os.getenv("MIC_DEVICE", "default").lower()

        # Read number of mic chanels from environment (MIC_CHANELS)
        self.mic_channels = os.getenv("MIC_CHANELS", 1)

        # Audio parameters
        self.chunk_size = 1024  # Buffer size
        self.audio_format = pyaudio.paInt16  # 16-bit audio format
        self.sample_rate = None  # Sample rate in Hz. Will be determined dynamically
        self.input_device_index = None  # Will be determined dynamically

        # ROS2 publisher for raw audio chunks
        self.ros_publisher = self.create_publisher(Int16MultiArray, "audio_stream", 10)

        self.py_audio = pyaudio.PyAudio()
        self.select_input_device()

        if self.input_device_index is None:
            self.get_logger().error("No audio input device found; shutting down.")
            rclpy.shutdown()
            return

        # Configure mic device configuration
        selected_input_device_info = self.py_audio.get_device_info_by_index(
            self.input_device_index
        )
        self.sample_rate = int(selected_input_device_info.get("defaultSampleRate", -1))

        self.get_logger().info(
            f"Device info: {self.sample_rate}Hz, {self.mic_channels} channels"
        )

        if self.sample_rate == -1:
            self.get_logger().error(
                "No audio counfiguration data found; shutting down."
            )
            rclpy.shutdown()
            return

        # Open audio stream
        self.audio_stream = self.py_audio.open(
            format=self.audio_format,
            channels=self.mic_channels,
            rate=self.sample_rate,
            input=True,
            input_device_index=self.input_device_index,
            frames_per_buffer=self.chunk_size,
        )

        # Schedule callback every chunk/rate seconds for minimal latency
        self.timer = self.create_timer(
            self.chunk_size / self.sample_rate, self.publish_audio
        )

    def select_input_device(self):
        """Select the user-preferred audio device or fall back to default."""
        found_device = None
        for i in range(self.py_audio.get_device_count()):
            found_device_info = self.py_audio.get_device_info_by_index(i)
            self.get_logger().info(
                f"Device {i}: {found_device_info.get('name')} (in:{found_device_info.get('maxInputChannels')}"
            )
            if self.mic_preferred_name in found_device_info.get("name").lower():
                found_device = (i, found_device_info)
                break

        if found_device:
            found_device_index, found_device_info = found_device
            self.input_device_index = found_device_index
            self.get_logger().info(
                f"Using preferred mic '{self.mic_preferred_name}': {found_device_info.get('name')} (index {found_device_index})"
            )
        else:
            # Fallback to system default input
            try:
                default_input_device_info = (
                    self.py_audio.get_default_input_device_info()
                )
                self.input_device_index = int(default_input_device_info.get("index"))
                self.get_logger().warn(
                    f"No device matching '{self.mic_preferred_name}'; falling back to default: "
                    f"{default_input_device_info.get('name')} (index {self.input_device_index})"
                )
            except IOError:
                self.get_logger().error(
                    f"No device matching '{self.mic_preferred_name}' and no default input; shutting down."
                )
                self.input_device_index = None

    def publish_audio(self):
        """Read audio data from the microphone and publish it."""
        if self.input_device_index is None:
            return

        audio_data = np.frombuffer(
            self.audio_stream.read(self.chunk_size, exception_on_overflow=False),
            dtype=np.int16,
        )
        ros_message = Int16MultiArray()
        ros_message.data = audio_data.tolist()
        self.ros_publisher.publish(ros_message)

    def destroy_node(self):
        """Cleanup resources when shutting down."""
        self.audio_stream.stop_stream()
        self.audio_stream.close()
        self.py_audio.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AudioStreamer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
