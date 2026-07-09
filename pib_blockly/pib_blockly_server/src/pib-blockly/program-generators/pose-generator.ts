import {Block} from "blockly/core/block";
import {pythonGenerator} from "blockly/python";
import {
    CONFIGURE_LOGGING,
    IMPORT_LOGGING,
    IMPORT_RCLPY,
    IMPORT_SYS,
    IMPORT_APPLY_JOINT_TRAJECTORY,
    IMPORT_JOINT_TRAJECTORY_MESSAGES,
    INIT_ROS,
    INIT_APPLY_JOINT_TRASJECTORY_CLIENT,
    IMPORT_POSE_CLIENT
} from "./util/definitions";
import {APPLY_POSE_FUNCTION} from "./util/function-declarations";

export function moveToPoseGenerator(
    block: Block,
    generator: typeof pythonGenerator,
) {
    // extract block-input
    const poseId = <string>block.getFieldValue("POSE");

    // add definitions to generator
    Object.assign(generator.definitions_, {
        IMPORT_RCLPY,
        IMPORT_SYS,
        IMPORT_LOGGING,
        IMPORT_APPLY_JOINT_TRAJECTORY,
        IMPORT_JOINT_TRAJECTORY_MESSAGES,
        CONFIGURE_LOGGING,
        INIT_ROS,
        IMPORT_POSE_CLIENT,
        INIT_APPLY_JOINT_TRASJECTORY_CLIENT
    });

    // declare the 'apply_pose'-function
    const functionName = generator.provideFunction_(
        "apply_pose",
        APPLY_POSE_FUNCTION(generator),
    );

    return `${functionName}("${poseId}")\n`;
}

export {pythonGenerator};
