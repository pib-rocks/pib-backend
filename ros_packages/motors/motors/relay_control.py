import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.service import Service
from rclpy.executors import SingleThreadedExecutor
from pib_motors.bricklet import set_ssr_state, solid_state_relay_bricklet
from datatypes.msg import SolidStateRelayState
from datatypes.srv import SetSolidStateRelay


class RelayControl(Node):

    def __init__(self):
        super().__init__("relay_control")
        self.state: SolidStateRelayState = SolidStateRelayState()
        self.state.turned_on = False
        self.relay = solid_state_relay_bricklet
        self.relay_available = True

        # Publisher for solid state relay status
        self.relay_state_publisher: Publisher = self.create_publisher(
            SolidStateRelayState, "solid_state_relay_state", 10
        )

        # Service for setting the solid state relay state
        self.set_voice_assistant_service: Service = self.create_service(
            SetSolidStateRelay,
            "set_solid_state_relay_state",
            self.set_solid_state_relay_state,
        )

        self.polling_timer = self.create_timer(1.0, self.poll_relay_state)

        self.get_logger().info("Now Running RELAY_CONTROL")

    def set_solid_state_relay_state(
        self, request: SetSolidStateRelay.Request, response: SetSolidStateRelay.Response
    ):
        """callback function for 'set_solid_state_relay_state' service"""
        request_state: SolidStateRelayState = request.solid_state_relay_state
        successful = self.update_relay_state(request_state.turned_on)
        response.successful = successful
        return response

    def update_relay_state(self, turned_on: bool) -> bool:
        """attempts to update the solid state relay, and returns whether this was successful"""
        if self.state.turned_on == turned_on:
            return True
        try:
            set_ssr_state(turned_on)
            self.state.turned_on = turned_on
        except Exception as e:
            self.get_logger().error(
                f"following error occured while trying to update solid state relay state: {str(e)}."
            )
            return False

        relay_state = SolidStateRelayState()
        relay_state.turned_on = self.state.turned_on

        self.relay_state_publisher.publish(relay_state)
        return True

    def poll_relay_state(self):
        if self.relay is None:
            return
        try:
            current_relay_state = self.relay.get_state()
            self.relay_available = True
        except Exception as e:
            if self.relay_available:
                self.get_logger().error(f"Error getting relay state: {str(e)}")
            self.relay_available = False
            return

        if current_relay_state is not self.state.turned_on:
            self.state.turned_on = current_relay_state

            msg = SolidStateRelayState()
            msg.turned_on = self.state.turned_on
            self.relay_state_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RelayControl()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
