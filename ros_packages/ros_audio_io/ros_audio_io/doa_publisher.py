import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import usb.core
import usb.util
import time
import os

from ros_audio_io.tuning import Tuning

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
            try:
                interval = float(os.getenv('DOA_PUBLISH_INTERVAL', '2.0'))
            except ValueError:
                self.get_logger().warn("Invalid DOA_PUBLISH_INTERVAL, defaulting to 2.0s")
                interval = 2.0

            self.get_logger().info(f"DOA publishing every {interval:.3f}s")
            self.timer = self.create_timer(interval, self.publish_doa)
            

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
