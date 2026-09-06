import {Block} from "blockly/core/block";
import {Order, pythonGenerator} from "blockly/python";
import {
    CONFIGURE_LOGGING,
    IMPORT_LOGGING,
    IMPORT_OS,
    IMPORT_PIB_SDK,
    IMPORT_PIB_SDK_BACKEND,
    IMPORT_PIB_SDK_POSE_CONTROL,
    IMPORT_PIB_SDK_POSES,
    IMPORT_PIB_SDK_TELEMETRY,
    IMPORT_POSE_CLIENT,
    IMPORT_RCLPY,
    IMPORT_SYS,
    IMPORT_URLPARSE,
    INIT_PIB_SDK_POSE_BACKEND,
    INIT_ROS,
} from "./util/definitions";
import {
    APPLY_POSE_FUNCTION,
    SAVE_CURRENT_POSE_FUNCTION,
} from "./util/function-declarations";

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
        IMPORT_OS,
        IMPORT_LOGGING,
        IMPORT_PIB_SDK,
        IMPORT_POSE_CLIENT,
        CONFIGURE_LOGGING,
        INIT_ROS,
    });

    // declare the 'apply_pose'-function
    const functionName = generator.provideFunction_(
        "apply_pose",
        APPLY_POSE_FUNCTION(generator),
    );

    return `${functionName}("${poseId}")\n`;
}

function addPoseSdkDefinitions(generator: typeof pythonGenerator) {
    Object.assign(generator.definitions_, {
        IMPORT_OS,
        IMPORT_PIB_SDK_POSES,
        IMPORT_PIB_SDK_BACKEND,
        IMPORT_URLPARSE,
        INIT_PIB_SDK_POSE_BACKEND,
    });
}

function quotedField(
    block: Block,
    generator: typeof pythonGenerator,
    fieldName: string,
) {
    return generator.quote_(String(block.getFieldValue(fieldName) || ""));
}

export function save_current_pose(
    block: Block,
    generator: typeof pythonGenerator,
) {
    addPoseSdkDefinitions(generator);
    Object.assign(generator.definitions_, {
        IMPORT_PIB_SDK_POSE_CONTROL,
        IMPORT_PIB_SDK_TELEMETRY,
    });

    const functionName = generator.provideFunction_(
        "save_current_pose_with_all_motors",
        SAVE_CURRENT_POSE_FUNCTION(generator),
    );

    return `${functionName}(${quotedField(block, generator, "NAME")})\n`;
}

export function get_all_poses(
    _block: Block,
    generator: typeof pythonGenerator,
): [string, Order] {
    addPoseSdkDefinitions(generator);
    return ["list_poses(pose_backend)", Order.FUNCTION_CALL];
}

export function get_pose_joints(
    block: Block,
    generator: typeof pythonGenerator,
): [string, Order] {
    addPoseSdkDefinitions(generator);
    const name = quotedField(block, generator, "NAME");
    return [
        `get_pose(pose_backend, name=${name}).motor_angles_deg`,
        Order.MEMBER,
    ];
}

export function has_pose(
    block: Block,
    generator: typeof pythonGenerator,
): [string, Order] {
    addPoseSdkDefinitions(generator);
    const name = quotedField(block, generator, "NAME");
    return [
        `any(pose.name == ${name} for pose in list_poses(pose_backend))`,
        Order.FUNCTION_CALL,
    ];
}

export function pose_count(
    _block: Block,
    generator: typeof pythonGenerator,
): [string, Order] {
    addPoseSdkDefinitions(generator);
    return ["len(list_poses(pose_backend))", Order.FUNCTION_CALL];
}

export {pythonGenerator};
