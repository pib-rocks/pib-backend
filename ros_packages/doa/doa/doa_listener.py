# It is template example, to get doa data from ros2 stream
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Int16MultiArray
import numpy as np

class DOAListener(Node):
    def __init__(self):
        super().__init__('doa_listener')

        # Subscribe to the DOA topic
        self.subscription = self.create_subscription(
            Int32, 
            'doa_angle', 
            self.listener_callback, 
            10
        )

    def listener_callback(self, msg):
        direction = msg.data
        self.get_logger().info(f'Received DOA angle: {direction}°')

def main(args=None):
    rclpy.init(args=args)
    node = DOAListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()