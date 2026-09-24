"""Motor parameter data shared by all educational hardware profiles."""

MOTOR_PARAMETER_DEFAULTS = {
    "pulse_width_min": 700,
    "pulse_width_max": 2500,
    "rotation_range_min": -9000,
    "rotation_range_max": 9000,
    "velocity": 16000,
    "acceleration": 10000,
    "deceleration": 5000,
    "period": 19500,
    "turned_on": True,
    "visible": True,
    "invert": False,
}

FINGER_MOTOR_PARAMETERS = {
    "pulse_width_min": 750,
    "velocity": 100000,
    "acceleration": 50000,
    "deceleration": 50000,
}

MOTOR_PARAMETER_DEVIATIONS = {
    "tilt_forward_motor": {
        "rotation_range_min": -4500,
        "rotation_range_max": 4500,
    },
    "upper_arm_left_rotation": {"velocity": 10000},
    "upper_arm_right_rotation": {"velocity": 10000},
    "thumb_right_opposition": FINGER_MOTOR_PARAMETERS,
    "thumb_right_stretch": FINGER_MOTOR_PARAMETERS,
    "index_right_stretch": FINGER_MOTOR_PARAMETERS,
    "middle_right_stretch": FINGER_MOTOR_PARAMETERS,
    "ring_right_stretch": FINGER_MOTOR_PARAMETERS,
    "pinky_right_stretch": FINGER_MOTOR_PARAMETERS,
    "thumb_left_opposition": FINGER_MOTOR_PARAMETERS,
    "thumb_left_stretch": FINGER_MOTOR_PARAMETERS,
    "index_left_stretch": FINGER_MOTOR_PARAMETERS,
    "middle_left_stretch": FINGER_MOTOR_PARAMETERS,
    "ring_left_stretch": FINGER_MOTOR_PARAMETERS,
    "pinky_left_stretch": FINGER_MOTOR_PARAMETERS,
}
