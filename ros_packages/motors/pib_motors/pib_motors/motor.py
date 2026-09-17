from typing import Any
from pib_motors.actuator import Actuator, ServoBrickletActuator, create_actuator
from pib_api_client import motor_client


class Motor:

    MIN_ROTATION: int = -9000
    MAX_ROTATION: int = 9000

    NO_CURRENT: int = ServoBrickletActuator.NO_CURRENT

    def __init__(self, name: str, actuators: list[Actuator], invert: bool):
        self.name: str = name
        self.visible: bool = True
        self.actuators: list[Actuator] = actuators
        self.invert: bool = invert
        self.rotation_range_min: int = Motor.MIN_ROTATION
        self.rotation_range_max: int = Motor.MAX_ROTATION

    @property
    def bricklet_pins(self) -> list[Actuator]:
        """transition alias for 'actuators', for consumers not yet moved over"""
        return self.actuators

    def __str__(self):
        return f"MOTOR[ bricklet_pins: {[str(bp) for bp in self.actuators]}, settings: {self.get_settings()} ]"

    def apply_settings(self, settings_dto: dict[str, Any]) -> bool:
        """apply provided settings to the motor"""
        self.visible = settings_dto["visible"]
        self.invert = settings_dto["invert"]
        self.rotation_range_min = settings_dto["rotationRangeMin"]
        self.rotation_range_max = settings_dto["rotationRangeMax"]

        if not self.actuators:
            return False

        # Check if current position is outside of new rotation Ranges
        adjusted_position = self._validate_position(self.get_position())
        if adjusted_position != self.get_position():
            self.set_position(adjusted_position)

        return all(actuator.apply_settings(settings_dto) for actuator in self.actuators)

    def get_settings(self) -> dict[str, Any]:
        """get the current settings of this motor"""
        settings = {
            "visible": self.visible,
            "name": self.name,
            "invert": self.invert,
            "rotationRangeMin": self.rotation_range_min,
            "rotationRangeMax": self.rotation_range_max,
        }
        if not self.actuators:
            return settings
        settings.update(self.actuators[0].get_settings())
        return settings

    def set_position(self, position: int) -> bool:
        """sets the position of all actuators associated with this motor"""
        if not self.actuators:
            return False
        if self.invert:
            position *= -1
        position = self._validate_position(position)
        return all(actuator.set_position(position) for actuator in self.actuators)

    def get_position(self) -> int:
        """returns the target position of the motor as set by the last command, or '0' if no actuator is connected"""
        if not self.actuators:
            return 0
        return self.actuators[0].get_position()

    def get_current_position(self) -> int:
        """returns the actual physical position of the motor at this moment, or '0' if no actuator is connected"""
        if not self.actuators:
            return 0
        return self.actuators[0].get_current_position()

    def has_reached_position(self) -> bool:
        """returns 'True' if all actuators of this motor have reached their target position"""
        if not self.actuators:
            return True
        return all(actuator.has_reached_target() for actuator in self.actuators)

    def get_current(self) -> int:
        """returns the maximum current of all actuators, or NO_CURRENT, if no actuator is connected"""
        if not self.actuators:
            return Motor.NO_CURRENT
        return max(actuator.get_current() for actuator in self.actuators)

    def check_if_motor_is_connected(self) -> bool:
        """returns 'True' if all actuators of this motor are connected"""
        return bool(self.actuators) and all(
            actuator.is_connected() for actuator in self.actuators
        )

    def _validate_position(self, position: int) -> int:
        """Check if position is within range, set it to the min/max value if not."""
        position = min(max(position, self.rotation_range_min), self.rotation_range_max)
        return position


# get data from pib-api
successful, response = motor_client.get_all_motors()
if not successful:
    raise RuntimeError("failed to load motors from pib-api...")

# list of all available motor-objects
motors: list[Motor] = []
for motor_dto in response["motors"]:
    actuators = [
        create_actuator(
            ServoBrickletActuator.kind,
            bricklet_pin_dto["pin"],
            bricklet_pin_dto["bricklet"],
            bricklet_pin_dto["invert"],
        )
        for bricklet_pin_dto in motor_dto["brickletPins"]
        if bricklet_pin_dto["bricklet"]
    ]
    motors.append(Motor(motor_dto["name"], actuators, motor_dto["invert"]))

# maps the name of a (multi-)motor to its associated motor objects
name_to_motors: dict[str, Motor] = {motor.name: [motor] for motor in motors}
name_to_motors["all_fingers_left"] = [
    motor for motor in motors if motor.name.endswith("left_stretch")
]
name_to_motors["all_fingers_right"] = [
    motor for motor in motors if motor.name.endswith("right_stretch")
]
