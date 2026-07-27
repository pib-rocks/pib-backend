import {Block} from "blockly/core/block";
import {pythonGenerator} from "blockly/python";
import {
    CONFIGURE_LOGGING,
    IMPORT_LOGGING,
    IMPORT_PIB_SDK,
    IMPORT_POSE_CLIENT,
    IMPORT_RCLPY,
    IMPORT_SYS,
    INIT_ROS,
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
        IMPORT_PIB_SDK,
        CONFIGURE_LOGGING,
        INIT_ROS,
        IMPORT_POSE_CLIENT,
    });

    // declare the 'apply_pose'-function
    const functionName = generator.provideFunction_(
        "apply_pose",
        APPLY_POSE_FUNCTION(generator),
    );

    return `${functionName}("${poseId}")\n`;
}

export {pythonGenerator};
