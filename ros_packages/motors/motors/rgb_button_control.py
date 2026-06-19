import rclpy

from rclpy.node import Node
from rclpy.client import Client
from rclpy.task import Future
from rclpy.service import Service
from tinkerforge.bricklet_rgb_led_button import BrickletRGBLEDButton
from pib_motors.bricklet import uid_to_rgb_led_bricklet
from datatypes.srv import ProxyRunProgramStart, ProxyRunProgramStop, SetButtonColor
from datatypes.msg import ProxyRunProgramResult
from pib_api_client import button_programs_client

ERROR_COLOR_DURATION_SECONDS = 2.0  # how long red stays before reverting to blue
BLUE_COLOR = (0, 0, 255)  # blue color for idle state
GREEN_COLOR = (0, 255, 0)  # green color for program running state
RED_COLOR = (255, 0, 0)  # red color for error state


class RGBButtonControl(Node):
    def __init__(self):
        super().__init__("rgb_button_control")
        self.rgb_led_bricklets: dict[str, BrickletRGBLEDButton] = (
            uid_to_rgb_led_bricklet
        )
        if not self.rgb_led_bricklets:
            self.get_logger().info("No RGB Button Bricklet found")

        for uid, bricklet in self.rgb_led_bricklets.items():
            try:
                bricklet.register_callback(
                    BrickletRGBLEDButton.CALLBACK_BUTTON_STATE_CHANGED,
                    lambda state, uid=uid: self.on_button_state_changed(uid, state),
                )
            except Exception as e:
                self.get_logger().error(
                    f"Error registering callback for RGB Button Bricklet with UID {uid}: {str(e)}"
                )

        # Service for setting the button color
        self.set_button_color_service: Service = self.create_service(
            SetButtonColor,
            "set_button_color",
            self.set_button_color_callback,
        )

        self.start_program_client: Client = self.create_client(
            ProxyRunProgramStart, "proxy_run_program_start"
        )
        self.start_program_client.wait_for_service()

        self.stop_program_client: Client = self.create_client(
            ProxyRunProgramStop, "proxy_run_program_stop"
        )
        self.stop_program_client.wait_for_service()

        self.program_result_subscriber = self.create_subscription(
            ProxyRunProgramResult,
            "proxy_run_program_result",
            self.program_result_callback,
            10,
        )

        self.goal_to_uid: dict[str, str] = {}

        self.set_button_startup_color()

        self.get_logger().info("Now Running RGB_BUTTON_CONTROL")

    def set_button_color_callback(
        self, request: SetButtonColor.Request, response: SetButtonColor.Response
    ):
        """callback function for 'set_button_color' service"""
        uid = request.uid
        r = request.r
        g = request.g
        b = request.b

        successful = self.set_button_color(uid, r, g, b)
        response.successful = successful
        return response

    def program_result_callback(self, msg: ProxyRunProgramResult) -> None:
        """Set button color to red on error (temporary), blue on success."""
        uid = self.goal_to_uid.pop(msg.proxy_goal_id, None)
        if not uid:
            return

        if msg.exit_code != 0:
            self.set_button_color(uid, *RED_COLOR)
            timer = self.create_timer(
                ERROR_COLOR_DURATION_SECONDS,
                lambda: (self.set_button_color(uid, *BLUE_COLOR), timer.cancel()),
            )
            self.get_logger().warning(
                f"Program failed for UID {uid}, exit_code={msg.exit_code}"
            )
        else:
            self.set_button_color(uid, *BLUE_COLOR)

    def on_button_state_changed(self, uid: str, state) -> None:
        """Callback function for button press events."""
        if state == BrickletRGBLEDButton.BUTTON_STATE_PRESSED:
            # If program is running for this button, stop it
            running_goal_id = self.get_running_goal_id(uid)
            if running_goal_id:
                self.stop_program(running_goal_id, uid)
                return

            self.load_button_programs()
            program_number = self.uid_to_program.get(uid)
            if program_number:
                self.start_program(program_number, uid)
            else:
                self.get_logger().warning(
                    f"No program assigned to RGB Button with UID {uid}."
                )

    def get_running_goal_id(self, uid: str) -> str | None:
        """Returns the proxy_goal_id if a program is currently running for this button."""
        for goal_id, goal_uid in self.goal_to_uid.items():
            if goal_uid == uid:
                return goal_id
        return None

    def stop_program(self, proxy_goal_id: str, uid: str) -> None:
        """Stops a running program by proxy_goal_id."""
        request = ProxyRunProgramStop.Request()
        request.proxy_goal_id = proxy_goal_id
        future: Future = self.stop_program_client.call_async(request)

        def on_response(fut: Future):
            response = fut.result()
            if response:
                self.goal_to_uid.pop(proxy_goal_id, None)
                self.set_button_color(uid, *BLUE_COLOR)

        future.add_done_callback(on_response)

    def load_button_programs(self) -> None:
        """Loads the button programs from the pib-api"""
        successful, button_programs_dto = button_programs_client.get_button_programs()
        if not successful:
            self.get_logger().error("Failed to load button programs from backend.")
            return

        self.uid_to_program = {}
        for button_program in button_programs_dto["buttonPrograms"]:
            uid = button_program.get("brickletUid")
            if uid:
                self.uid_to_program[uid] = button_program.get("programNumber")

    def start_program(self, program_number: str, uid: str) -> None:
        """Starts a program and stores the goal_id to uid mapping."""
        request = ProxyRunProgramStart.Request()
        request.program_number = program_number
        self.set_button_color(uid, *GREEN_COLOR)
        future: Future = self.start_program_client.call_async(request)

        def on_response(fut: Future):
            response = fut.result()
            if response:
                self.goal_to_uid[response.proxy_goal_id] = uid

        future.add_done_callback(on_response)

    def set_button_color(self, uid: str, r: int, g: int, b: int) -> bool:
        """Sets the LED color of the RGB Button with the given UID."""
        bricklet = self.rgb_led_bricklets.get(uid)
        if not bricklet:
            return False
        try:
            bricklet.set_color(r, g, b)
            return True
        except Exception as e:
            self.get_logger().error(f"Error setting color for UID {uid}: {str(e)}")
            return False

    def set_button_startup_color(self) -> None:
        """Sets button colors based on program assignments on startup."""
        self.load_button_programs()
        for uid in self.rgb_led_bricklets:
            self.set_button_color(uid, *BLUE_COLOR)


def main(args=None):

    rclpy.init(args=args)
    rgb_led_control = RGBButtonControl()
    rclpy.spin(rgb_led_control)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
