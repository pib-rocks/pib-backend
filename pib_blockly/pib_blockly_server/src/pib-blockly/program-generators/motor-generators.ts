import {Block} from "blockly/core/block";
import {Order, pythonGenerator} from "blockly/python";
import {
    CONFIGURE_LOGGING,
    IMPORT_APPLY_JOINT_TRAJECTORY,
    IMPORT_GET_JOINT_POSITION,
    IMPORT_JOINT_TRAJECTORY_MESSAGES,
    IMPORT_LOGGING,
    IMPORT_RCLPY,
    IMPORT_SYS,
    INIT_APPLY_JOINT_TRASJECTORY_CLIENT,
    INIT_GET_JOINT_POSITION_CLIENT,
    INIT_ROS,
} from "./util/definitions";
import {APPLY_JOINT_TRAJECTORY_FUNCTION, GET_JOINT_POSITION_FUNCTION} from "./util/function-declarations";

const motorOptionToMotorName = new Map()
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

export function move_motor(block: Block, generator: typeof pythonGenerator) {
    // extract block-input
    const motorOption = <string>block.getFieldValue("MOTORNAME");
    const modeInput = block.getFieldValue("MODE");
    const positionInput = String(
        generator.valueToCode(block, "POSITION", Order.ATOMIC),
    );
    const selectedMotorName: string = motorOptionToMotorName.get(motorOption);
    if (selectedMotorName === undefined) {
        throw new Error(
            `'${selectedMotorName}' is not a valid value for 'MOTORNAME'.`,
        );
    }

    // add definitions to generator
    Object.assign(generator.definitions_, {
        IMPORT_RCLPY,
        IMPORT_SYS,
        IMPORT_LOGGING,
        IMPORT_JOINT_TRAJECTORY_MESSAGES,
        IMPORT_APPLY_JOINT_TRAJECTORY,
        CONFIGURE_LOGGING,
        INIT_ROS,
        INIT_APPLY_JOINT_TRASJECTORY_CLIENT,
    });

    if (modeInput == "RELATIVE") {
        Object.assign(generator.definitions_, {
            IMPORT_GET_JOINT_POSITION,
            INIT_GET_JOINT_POSITION_CLIENT,
        });
    }

    // declare the 'apply_joint_trajectory'-function
    const functionName = generator.provideFunction_(
        "apply_joint_trajectory",
        APPLY_JOINT_TRAJECTORY_FUNCTION(generator),
    );

    // generate code for computing the target postion of the selected motor
    let positionString = "";
    if (modeInput == "ABSOLUTE") {
        positionString = positionInput;
    } else if (modeInput == "RELATIVE") {
        const getPositionFunctionName = generator.provideFunction_(
            "get_joint_position",
            GET_JOINT_POSITION_FUNCTION(generator),
        );
        positionString = 
            `${getPositionFunctionName}('${selectedMotorName}') + ${positionInput}`;
    } else {
        throw new Error(`unexpected input-mode: ${modeInput}.`);
    }
    return `${functionName}("${selectedMotorName}", ${positionString})\n`;
}

export {pythonGenerator};
