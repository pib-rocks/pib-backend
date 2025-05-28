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
        self.preferred = os.getenv("MIC_DEVICE", "default").lower()

        # Audio parameters
        self.chunk = 1024  # Buffer size
        self.format = pyaudio.paInt16  # 16-bit audio format
        self.channels = 1  # Mono recording
        self.rate = 16000  # Sample rate in Hz
        self.input_device_index = None  # Will be determined dynamically

        # ROS2 publisher for raw audio chunks
        self.publisher_ = self.create_publisher(Int16MultiArray, "audio_stream", 10)

        self.audio = pyaudio.PyAudio()
        self.select_input_device()

        self.stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            input_device_index=self.input_device_index,
            frames_per_buffer=self.chunk,
        )

        # Schedule callback every chunk/rate seconds for minimal latency
        self.timer = self.create_timer(self.chunk / self.rate, self.publish_audio)

    def select_input_device(self):
        """Select the user-preferred audio device or fall back to default."""
        found = None
        for i in range(self.audio.get_device_count()):
            info = self.audio.get_device_info_by_index(i)
            self.get_logger().debug(
                f"Device {i}: {info['name']} (in:{info.get('maxInputChannels')}, out:{info.get('maxOutputChannels')})"
            )
            if self.preferred in info["name"].lower():
                found = (i, info)
                break

        if found:
            idx, info = found
            self.input_device_index = idx
            self.get_logger().info(
                f"Using preferred mic '{self.preferred}': {info['name']} (index {idx})"
            )
        else:
            # Fallback to system default input
            try:
                default = self.audio.get_default_input_device_info()
                self.input_device_index = int(default["index"])
                self.get_logger().warn(
                    f"No device matching '{self.preferred}'; falling back to default: "
                    f"{default['name']} (index {self.input_device_index})"
                )
            except IOError:
                self.get_logger().error(
                    f"No device matching '{self.preferred}' and no default input; shutting down."
                )
                self.input_device_index = None

    def publish_audio(self):
        """Read audio data from the microphone and publish it."""
        if self.input_device_index is None:
            return

        audio_data = np.frombuffer(
            self.stream.read(self.chunk, exception_on_overflow=False), dtype=np.int16
        )
        msg = Int16MultiArray()
        msg.data = audio_data.tolist()
        self.publisher_.publish(msg)
        # self.get_logger().info(f"Published {len(audio_data)} samples of audio data")

    def destroy_node(self):
        """Cleanup resources when shutting down."""
        self.stream.stop_stream()
        self.stream.close()
        self.audio.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AudioStreamer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
