import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
import pyaudio
import numpy as np


class AudioStreamer(Node):
    def __init__(self):
        super().__init__('audio_streamer')

        self.chunk = 1024  # Buffer size
        self.format = pyaudio.paInt16  # 16-bit audio format
        self.channels = 1  # Mono recording
        self.rate = 16000  # Sample rate in Hz
        self.input_device_index = None  # Will be determined dynamically

        self.publisher_ = self.create_publisher(Int16MultiArray, 'audio_stream', 10)

        self.audio = pyaudio.PyAudio()
        self.select_input_device()

        self.stream = self.audio.open(format=self.format, channels=self.channels,
                                      rate=self.rate, input=True,
                                      input_device_index=self.input_device_index,
                                      frames_per_buffer=self.chunk)

        self.timer = self.create_timer(0.1, self.publish_audio)

    def select_input_device(self):
        """Find and set the correct microphone input device, or fall back to the default."""
        # List all devices (optional, for debugging)
        for i in range(self.audio.get_device_count()):
            info = self.audio.get_device_info_by_index(i)
            self.get_logger().info(
                f"Device {i}: {info['name']} — "
                f"in:{info.get('maxInputChannels')} out:{info.get('maxOutputChannels')}"
            )

        # Try to pick any Respeaker (or similar) array first
        for i in range(self.audio.get_device_count()):
            info = self.audio.get_device_info_by_index(i)
            if "respeaker" in info['name'].lower():
                self.input_device_index = i
                self.get_logger().info(f"Selected audio input device: {info['name']} (index {i})")
                return

        # Fallback: use default input device
        try:
            default_info = self.audio.get_default_input_device_info()
            self.input_device_index = int(default_info['index'])
            self.get_logger().warn(
                f"No Respeaker found, falling back to default input device: "
                f"{default_info['name']} (index {self.input_device_index})"
            )
        except IOError:
            # No default device either
            self.get_logger().error("No valid microphone found and no default input device available!")
            self.input_device_index = None

    def publish_audio(self):
        """Read audio data from the microphone and publish it."""
        if self.input_device_index is None:
            return

        audio_data = np.frombuffer(self.stream.read(self.chunk, exception_on_overflow=False), dtype=np.int16)
        msg = Int16MultiArray()
        msg.data = audio_data.tolist()
        self.publisher_.publish(msg)
        #self.get_logger().info(f"Published {len(audio_data)} samples of audio data")

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


if __name__ == '__main__':
    main()
