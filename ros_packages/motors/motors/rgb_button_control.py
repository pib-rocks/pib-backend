import rclpy

from rclpy.node import Node
from pib_motors.bricklet import uid_to_rgb_led_bricklet
from tinkerforge.bricklet_rgb_led import BrickletRGBLEDButton


class RGBButtonControl(Node):
    def __init__(self):
        super().__init__("rgb_button_control")
        self.rgb_led_bricklets: dict[str, BrickletRGBLEDButton] = (
            uid_to_rgb_led_bricklet
        )
        if not self.rgb_led_bricklets:
            raise RuntimeError("No RGB Led Button Bricklet found")

        for uid, bricklet in self.rgb_led_bricklets.items():
            try:
                bricklet.register_callback(
                    BrickletRGBLEDButton.CALLBACK_BUTTON_PRESSED,
                    self.on_button_pressed,
                )
                self.get_logger().info(
                    f"Registered callback for RGB Button Bricklet with UID {uid}"
                )
            except Exception as e:
                self.get_logger().error(
                    f"Error registering callback for RGB Button Bricklet with UID {uid}: {str(e)}"
                )

    def on_button_pressed(self, state):
        """Callback function for button press events."""
        if state == BrickletRGBLEDButton.BUTTON_STATE_PRESSED:
            self.get_logger().info("Button pressed.")
        elif state == BrickletRGBLEDButton.BUTTON_STATE_RELEASED:
            self.get_logger().info("Button released.")


def main(args=None):

    rclpy.init(args=args)
    rgb_led_control = RGBButtonControl()
    rclpy.spin(rgb_led_control)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
