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
        """Find and set the correct microphone input device."""
        for i in range(self.audio.get_device_count()):
            device_info = self.audio.get_device_info_by_index(i)
            print(f"Device Index {i}: {device_info['name']}")
            print(f"  Max Input Channels: {device_info.get('maxInputChannels')}")
            print(f"  Max Output Channels: {device_info.get('maxOutputChannels')}\n")
            print(f"Device default sample rate: {device_info['defaultSampleRate']}\n")

        for i in range(self.audio.get_device_count()):
            device_info = self.audio.get_device_info_by_index(i)
            self.get_logger().info(f"Device {i}: {device_info['name']}")
            if "respeaker" in device_info['name'].lower():  # Adjust if using a different mic
                self.input_device_index = i
                self.get_logger().info(f"Selected audio input device: {device_info['name']}")
                print(f"Device {i}: {device_info['name']} - Default sample rate: {device_info['defaultSampleRate']}")
                return

        self.get_logger().error("No valid microphone found!")
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
