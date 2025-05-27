import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import usb.core
import usb.util
import time

# Add usb_4_mic_array to Python path so we can import tuning.py
sys.path.append('/ros2_ws/src/my_talker/my_talker/usb_4_mic_array')

from doa.tuning import Tuning

class DOAPublisher(Node):
    def __init__(self):
        super().__init__('doa_publisher')
        self.publisher_ = self.create_publisher(Int32, 'doa_angle', 10)

        # Find the ReSpeaker device
        self.dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)

        if self.dev is None:
            self.get_logger().error("ReSpeaker Mic Array v2.0 not found! Make sure it's connected.")
        else:
            self.Mic_tuning = Tuning(self.dev)
            self.timer = self.create_timer(2.0, self.publish_doa)

    def publish_doa(self):
        if self.dev is not None:
            direction = self.Mic_tuning.direction  # If it's a method, use self.Mic_tuning.direction()
            msg = Int32()
            msg.data = direction
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published DOA angle: {direction}")


def main(args=None):
    rclpy.init(args=args)
    node = DOAPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
