from typing import Any
from pib_motors.bricklet_pin import BrickletPin
from pib_api_client import motor_client


class Motor:

	def __init__(self, name: str, bricklet_pins: list[BrickletPin]):
		self.name: str = name
		self.visible: bool = True
		self.bricklet_pins = bricklet_pins

	def __str__(self):
		return f"MOTOR[ bricklet_pins: {[str(bp) for bp in self.bricklet_pins]}, settings: {self.get_settings()} ]"

	def apply_settings(self, settings_dto) -> bool:
		self.visible = settings_dto['visible']
		return all(bp.apply_settings(settings_dto) for bp in self.bricklet_pins)

	def get_settings(self) -> dict[str, Any]:
		motor_settings_dto = self.bricklet_pins[0].get_settings()
		motor_settings_dto['visible'] = self.visible
		motor_settings_dto['name'] = self.name
		return motor_settings_dto

	def set_position(self, position: int) -> bool:
		return all(bp.set_position(position) for bp in self.bricklet_pins)
	
	def get_position(self) -> int:
		return self.bricklet_pins[0].get_position()


# get data from pib-api
successful, response = motor_client.get_all_motors()
if not successful: raise Exception('failed to load motors from pib-api...')

# list of all available motor-objects
motors: list[Motor] = []
for motor_dto in response['motors']:
	bricklet_pins = [BrickletPin(bricklet_pin_dto['pin'], bricklet_pin_dto['bricklet']) for bricklet_pin_dto in motor_dto['brickletPins']]
	motors.append(Motor(motor_dto['name'], bricklet_pins))

# maps the name of a (multi-)motor to its associated motor objects
name_to_motors: dict[str, Motor] = { motor.name : [motor] for motor in motors }
name_to_motors['all_fingers_left'] = [ motor for motor in motors if motor.name.endswith('left_stretch') ]
name_to_motors['all_fingers_right'] = [ motor for motor in motors if motor.name.endswith('right_stretch') ]
