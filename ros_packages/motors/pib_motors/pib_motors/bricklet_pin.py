"""Compatibility module: the bricklet-pin is now the tinkerforge servo actuator.

Kept so that existing imports of ``BrickletPin`` keep working while consumers
move over to ``pib_motors.actuator``.
"""

from pib_motors.actuator import ServoBrickletActuator as BrickletPin

__all__ = ["BrickletPin"]
