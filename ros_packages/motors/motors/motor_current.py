import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from pib_api_client import motor_client
from pib_motors.motor import motors, Motor
from tinkerforge.bricklet_servo_v2 import BrickletServoV2

if not hasattr(BrickletServoV2, "CALLBACK_SERVO_CURRENT"):
    BrickletServoV2.CALLBACK_SERVO_CURRENT = 27


class MotorCurrent(Node):

    def __init__(self):
        super().__init__("motor_current")

        self.motor_current_publisher = self.create_publisher(
            DiagnosticStatus, "motor_current", 10
        )

        self.pin_to_motors = {}
        self.setup_callbacks()
        self.timer = self.create_timer(1.0, self.publish_motor_current)

        self.get_logger().info("Now Running MOTOR CURRENT")

    def setup_callbacks(self, motor_list=None):
        if motor_list is None:
            motor_list = motors

        self.pin_to_motors = {}
        connected_bricklets = set()

        for motor in motor_list:
            for actuator in motor.actuators:
                # this monitoring is Tinkerforge-specific: actuators of other families
                # carry no bricklet and are skipped here (a family-agnostic current
                # monitoring is a later step)
                bricklet = getattr(actuator, "bricklet", None)
                if actuator.is_connected() and bricklet is not None:
                    key = (id(bricklet), actuator.pin)
                    if key not in self.pin_to_motors:
                        self.pin_to_motors[key] = []
                    if motor.name not in self.pin_to_motors[key]:
                        self.pin_to_motors[key].append(motor.name)

                    connected_bricklets.add(bricklet)
                    self._configure_servo_current(bricklet, actuator.pin)

        for bricklet in connected_bricklets:
            bricklet.register_callback(
                BrickletServoV2.CALLBACK_SERVO_CURRENT,
                self._make_callback(bricklet),
            )

    def _configure_servo_current(self, bricklet, pin: int) -> None:
        try:
            bricklet.set_servo_current_configuration(pin, 100, value_has_to_change=True)
        except TypeError:
            try:
                bricklet.set_servo_current_configuration(pin, value_has_to_change=True)
            except TypeError:
                bricklet.set_servo_current_configuration(pin, 100)
        except Exception as e:
            self.get_logger().warn(
                f"Failed to configure current callback for pin {pin}: {e}"
            )

    def _make_callback(self, bricklet):
        def callback(servo_channel, current, *args, **kwargs):
            key = (id(bricklet), servo_channel)
            motor_names = self.pin_to_motors.get(key, [])
            for motor_name in motor_names:
                self.publish_diagnostic_status(motor_name, current)

        return callback

    def publish_motor_current(self):
        telemetry_batch: list[dict] = []
        for motor in motors:
            current = motor.get_current()
            if current != Motor.NO_CURRENT:
                extra = self._actuator_feedback(motor)
                self.publish_diagnostic_status(motor.name, current, extra)
            for actuator in getattr(motor, "actuators", None) or []:
                if getattr(actuator, "bricklet", None) is not None:
                    continue
                if not actuator.is_connected():
                    continue
                uid = getattr(actuator, "uid", None)
                get_voltage = getattr(actuator, "get_voltage", None)
                if not uid or not callable(get_voltage):
                    continue
                act_current = actuator.get_current()
                no_current = getattr(actuator, "NO_CURRENT", Motor.NO_CURRENT)
                if act_current == no_current:
                    continue
                voltage = get_voltage()
                telemetry_batch.append(
                    {
                        "uid": str(uid),
                        "pin": int(actuator.pin),
                        "voltage": float(voltage or 0.0),
                        "current": float(max(0, act_current)),
                    }
                )
        if telemetry_batch:
            try:
                motor_client.report_diagnostics_telemetry(telemetry_batch)
            except Exception:
                pass

    def _actuator_feedback(self, motor) -> dict[str, str]:
        actuators = getattr(motor, "actuators", None) or []
        if not actuators:
            return {}
        actuator = actuators[0]
        get_temp = getattr(actuator, "get_temperature", None)
        if not callable(get_temp):
            return {}
        extra: dict[str, str] = {}
        try:
            pos = motor.get_current_position()
            extra["actual_position"] = f"{pos / 100.0:.1f} °"
        except Exception:
            pass
        try:
            temp = get_temp()
            if temp is not None:
                extra["temperature"] = f"{temp} °C"
        except Exception:
            pass
        get_voltage = getattr(actuator, "get_voltage", None)
        if callable(get_voltage):
            try:
                voltage = get_voltage()
                if voltage is not None:
                    extra["voltage"] = f"{voltage:.1f} V"
            except Exception:
                pass
        return extra

    def publish_diagnostic_status(
        self,
        motor_name: str,
        current: int,
        extra: dict[str, str] | None = None,
    ) -> None:
        if current == Motor.NO_CURRENT:
            return
        msg = DiagnosticStatus()
        msg.level = DiagnosticStatus.WARN if current >= 1500 else DiagnosticStatus.OK
        msg.name = motor_name
        keyvalue = KeyValue()
        keyvalue.key = motor_name
        keyvalue.value = str(current)
        values = [keyvalue]
        if extra:
            for k, v in extra.items():
                kv = KeyValue()
                kv.key = k
                kv.value = v
                values.append(kv)
        msg.values = values
        self.motor_current_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotorCurrent()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
