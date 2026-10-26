export type MotorDropdownOption = [string, string];

export const MOTOR_NAME_DROPDOWN_OPTIONS: MotorDropdownOption[] = [
    ["thumb left opposition", "THUMB_LEFT_OPPOSITION"],
    ["thumb left stretch", "THUMB_LEFT_STRETCH"],
    ["index finger left stretch", "INDEX_LEFT_STRETCH"],
    ["middle finger left stretch", "MIDDLE_LEFT_STRETCH"],
    ["ring finger left stretch", "RING_LEFT_STRETCH"],
    ["pinky finger left stretch", "PINKY_LEFT_STRETCH"],
    ["all fingers left", "ALL_FINGERS_LEFT_STRETCH"],
    ["thumb right opposition", "THUMB_RIGHT_OPPOSITION"],
    ["thumb right stretch", "THUMB_RIGHT_STRETCH"],
    ["index finger right stretch", "INDEX_RIGHT_STRETCH"],
    ["middle finger right stretch", "MIDDLE_RIGHT_STRETCH"],
    ["ring finger right stretch", "RING_RIGHT_STRETCH"],
    ["pinky finger right stretch", "PINKY_RIGHT_STRETCH"],
    ["all fingers right", "ALL_FINGERS_RIGHT_STRETCH"],
    ["upper left arm rotation", "UPPER_ARM_LEFT_ROTATION"],
    ["elbow left", "ELBOW_LEFT"],
    ["lower left arm rotation", "LOWER_ARM_LEFT_ROTATION"],
    ["wrist left", "WRIST_LEFT"],
    ["left shoulder vertical", "SHOULDER_VERTICAL_LEFT"],
    ["left shoulder horizontal", "SHOULDER_HORIZONTAL_LEFT"],
    ["upper right arm rotation", "UPPER_ARM_RIGHT_ROTATION"],
    ["elbow right", "ELBOW_RIGHT"],
    ["lower right arm rotation", "LOWER_ARM_RIGHT_ROTATION"],
    ["wrist right", "WRIST_RIGHT"],
    ["right shoulder vertical", "SHOULDER_VERTICAL_RIGHT"],
    ["right shoulder horizontal", "SHOULDER_HORIZONTAL_RIGHT"],
    ["tilt head forward", "TILT_FORWARD_HEAD"],
    ["turn head", "TURN_HEAD"],
];

const MOTOR_OPTION_TO_SDK_NAME = new Map<string, string>()
    .set("THUMB_LEFT_OPPOSITION", "thumb_left_opposition")
    .set("THUMB_LEFT_STRETCH", "thumb_left_stretch")
    .set("INDEX_LEFT_STRETCH", "index_left_stretch")
    .set("MIDDLE_LEFT_STRETCH", "middle_left_stretch")
    .set("RING_LEFT_STRETCH", "ring_left_stretch")
    .set("PINKY_LEFT_STRETCH", "pinky_left_stretch")
    .set("ALL_FINGERS_LEFT_STRETCH", "all_fingers_left")
    .set("THUMB_RIGHT_OPPOSITION", "thumb_right_opposition")
    .set("THUMB_RIGHT_STRETCH", "thumb_right_stretch")
    .set("INDEX_RIGHT_STRETCH", "index_right_stretch")
    .set("MIDDLE_RIGHT_STRETCH", "middle_right_stretch")
    .set("RING_RIGHT_STRETCH", "ring_right_stretch")
    .set("PINKY_RIGHT_STRETCH", "pinky_right_stretch")
    .set("ALL_FINGERS_RIGHT_STRETCH", "all_fingers_right")
    .set("UPPER_ARM_LEFT_ROTATION", "upper_arm_left_rotation")
    .set("ELBOW_LEFT", "elbow_left")
    .set("LOWER_ARM_LEFT_ROTATION", "lower_arm_left_rotation")
    .set("WRIST_LEFT", "wrist_left")
    .set("SHOULDER_VERTICAL_LEFT", "shoulder_vertical_left")
    .set("SHOULDER_HORIZONTAL_LEFT", "shoulder_horizontal_left")
    .set("UPPER_ARM_RIGHT_ROTATION", "upper_arm_right_rotation")
    .set("ELBOW_RIGHT", "elbow_right")
    .set("LOWER_ARM_RIGHT_ROTATION", "lower_arm_right_rotation")
    .set("WRIST_RIGHT", "wrist_right")
    .set("SHOULDER_VERTICAL_RIGHT", "shoulder_vertical_right")
    .set("SHOULDER_HORIZONTAL_RIGHT", "shoulder_horizontal_right")
    .set("TILT_FORWARD_HEAD", "tilt_forward_motor")
    .set("TURN_HEAD", "turn_head_motor");

export function motorOptionToSdkName(option: string): string {
    const sdkName = MOTOR_OPTION_TO_SDK_NAME.get(option);
    if (sdkName === undefined) {
        throw new Error(`'${option}' is not a valid value for 'MOTORNAME'.`);
    }
    return sdkName;
}
