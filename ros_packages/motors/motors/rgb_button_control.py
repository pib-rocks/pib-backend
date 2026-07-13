import rclpy

from rclpy.node import Node
from rclpy.client import Client
from rclpy.task import Future
from tinkerforge.bricklet_rgb_led_button import BrickletRGBLEDButton
from pib_motors.bricklet import uid_to_rgb_led_bricklet
from datatypes.srv import ProxyRunProgramStart, ProxyRunProgramStop
from datatypes.msg import ProxyRunProgramResult
from pib_api_client import button_programs_client
from button_service.srv import SetButtonManualOverride

ERROR_COLOR_DURATION_SECONDS = 2.0  # how long red stays before reverting to blue
BLUE_COLOR = (0, 0, 255)  # blue color for idle state
GREEN_COLOR = (0, 255, 0)  # green color for program running state
RED_COLOR = (255, 0, 0)  # red color for error state
POLL_INTERVAL_SECONDS = 5.0


class RGBButtonControl(Node):
    def __init__(self):
        super().__init__("rgb_button_control")
        self.rgb_led_bricklets: dict[str, BrickletRGBLEDButton] = (
            uid_to_rgb_led_bricklet
        )
        if not self.rgb_led_bricklets:
            self.get_logger().info("No RGB Led Button Bricklet found")

        for uid, bricklet in self.rgb_led_bricklets.items():
            try:
                bricklet.register_callback(
                    BrickletRGBLEDButton.CALLBACK_BUTTON_STATE_CHANGED,
                    lambda state, uid=uid: self.on_button_state_changed(uid, state),
                )
                self.get_logger().info(
                    f"Registered callback for RGB Button Bricklet with UID {uid}"
                )
            except Exception as e:
                self.get_logger().error(
                    f"Error registering callback for RGB Button Bricklet with UID {uid}: {str(e)}"
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

        # maps proxy_goal_id -> uid
        self.goal_to_uid: dict[str, str] = {}

        # maps uid -> (r,g,b) for manual override colors (set via Blockly)
        self.manual_overrides: dict[str, tuple[int, int, int]] = {}

        self.manual_override_service = self.create_service(
            SetButtonManualOverride,
            "/rgb_button/manual_override",
            self.handle_manual_override,
        )

        self.update_button_colors()

        self.create_timer(POLL_INTERVAL_SECONDS, self.update_button_colors)

        self.get_logger().info("Now Running RGB_BUTTON_CONTROL")

    def handle_manual_override(self, request, response):
        uid = str(request.bricklet_uid)
        if not uid:
            response.success = False
            response.message = "bricklet_uid is required"
            return response

        if uid not in self.rgb_led_bricklets:
            response.success = False
            response.message = f"Unknown bricklet_uid {uid}"
            return response

        if request.enabled:
            r = int(request.red)
            g = int(request.green)
            b = int(request.blue)
            self.manual_overrides[uid] = (r, g, b)

            # If a program is running for this UID, program state has priority.
            # Otherwise apply override immediately.
            if uid not in self.goal_to_uid.values():
                self.set_button_color(uid, r, g, b)

            response.success = True
            response.message = "OK"
            return response

        self.manual_overrides.pop(uid, None)
        response.success = True
        response.message = "OK"
        return response

    def program_result_callback(self, msg: ProxyRunProgramResult) -> None:
        """Set button color to red on error (temporary), blue on success."""
        uid = self.goal_to_uid.pop(msg.proxy_goal_id, None)
        if not uid:
            return

        if msg.exit_code != 0:
            self.set_button_color(uid, *RED_COLOR)
            self.get_logger().warning(
                f"Program failed for UID {uid}, exit_code={msg.exit_code}"
            )
        else:
            self.set_button_color(uid, *BLUE_COLOR)
            self.get_logger().info(f"Program finished successfully for UID {uid}")

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
                self.get_logger().info(
                    f"Starting program {program_number} for button with UID {uid}."
                )
                self.start_program(program_number, uid)
            else:
                self.get_logger().warning(
                    f"No program assigned to button with UID {uid}."
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
                self.get_logger().info(
                    f"Stopped program for UID {uid}, proxy_goal_id={proxy_goal_id}"
                )

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
        request = ProxyRunProgramStart.Request()
        request.program_number = program_number
        self.set_button_color(uid, *GREEN_COLOR)
        future: Future = self.start_program_client.call_async(request)

        def on_response(fut: Future):
            response = fut.result()
            if response:
                self.goal_to_uid[response.proxy_goal_id] = uid
                self.get_logger().info(
                    f"Started program {program_number} for UID {uid}, proxy_goal_id={response.proxy_goal_id}"
                )

        future.add_done_callback(on_response)

    def set_button_color(self, uid: str, r: int, g: int, b: int) -> None:
        bricklet = self.rgb_led_bricklets.get(uid)
        if not bricklet:
            return
        try:
            bricklet.set_color(r, g, b)
        except Exception as e:
            self.get_logger().error(f"Error setting color for UID {uid}: {str(e)}")

    def update_button_colors(self) -> None:
        """Periodically updates button colors based on current program assignments."""
        self.load_button_programs()
        for uid in self.rgb_led_bricklets:
            if uid in self.goal_to_uid.values():
                continue  # don't override color while program is running
            if uid in self.manual_overrides:
                continue  # respect manual override (e.g. Blockly set-color)
            if self.uid_to_program.get(uid):
                self.set_button_color(uid, *BLUE_COLOR)
            else:
                self.set_button_color(uid, 0, 0, 0)


def main(args=None):

    rclpy.init(args=args)
    rgb_led_control = RGBButtonControl()
    rclpy.spin(rgb_led_control)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
