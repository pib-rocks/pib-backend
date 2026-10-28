import {Block} from "blockly/core/block";
import {Order, pythonGenerator} from "blockly/python";
import {CONFIGURE_LOGGING, IMPORT_GET_JOINT_POSITION, IMPORT_LOGGING, IMPORT_OS, IMPORT_PIB_SDK, IMPORT_PIB_SDK_IK, IMPORT_PIB_SDK_REP103, IMPORT_PIB_SDK_TELEMETRY, IMPORT_RCLPY, IMPORT_SYS, INIT_GET_JOINT_POSITION_CLIENT, INIT_ROS, addDefinitions} from "./util/definitions";
import {motorOptionToSdkName} from "../program-blocks/motor-motor-options";
import {
    APPLY_JOINT_TRAJECTORY_FUNCTION,
    GET_HAND_POSITION_REP103_FUNCTION,
    CENTIDEGREES_TO_DEGREES_FUNCTION,
    GET_JOINT_POSITION_FUNCTION,
    GET_MOTOR_CURRENT_MA_FUNCTION,
    MOVE_MOTORS_FUNCTION,
    SET_HAND_POSITION_XYZ_FUNCTION,
} from "./util/function-declarations";

const IMPORT_PIB_SDK_GET_HAND_POSITION_XYZ =
    "from pib_sdk import get_hand_position_xyz";

type MoveMotorsBlock = Block & {
    getMotorRowCount?: () => number;
    itemCount_?: number;
};

function moveMotorsRowCount(block: MoveMotorsBlock): number {
    if (typeof block.getMotorRowCount === "function") {
        return block.getMotorRowCount();
    }
    if (typeof block.itemCount_ === "number") {
        return block.itemCount_;
    }
    let count = 0;
    while (block.getInput(`ANGLE_${count}`)) {
        count++;
    }
    return count;
}

export function move_motor(block: Block, generator: typeof pythonGenerator) {
    // extract block-input
    const motorOption = <string>block.getFieldValue("MOTORNAME");
    const modeInput = block.getFieldValue("MODE");
    const positionInput = String(
        generator.valueToCode(block, "POSITION", Order.ATOMIC),
    );
    const selectedMotorName = motorOptionToSdkName(motorOption);

    // add definitions to generator
    addDefinitions(generator, {
        IMPORT_RCLPY,
        IMPORT_SYS,
        IMPORT_OS,
        IMPORT_LOGGING,
        IMPORT_PIB_SDK,
        CONFIGURE_LOGGING,
        INIT_ROS,
    });

    if (modeInput == "RELATIVE") {
        addDefinitions(generator, {
            IMPORT_GET_JOINT_POSITION,
            INIT_GET_JOINT_POSITION_CLIENT,
        });
        generator.provideFunction_(
            "centidegrees_to_degrees",
            CENTIDEGREES_TO_DEGREES_FUNCTION(generator),
        );
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
        positionString = `${getPositionFunctionName}('${selectedMotorName}') + ${positionInput}`;
    } else {
        throw new Error(`unexpected input-mode: ${modeInput}.`);
    }
    return `${functionName}("${selectedMotorName}", ${positionString})\n`;
}

export function move_motors(
    block: MoveMotorsBlock,
    generator: typeof pythonGenerator,
) {
    const rowCount = moveMotorsRowCount(block);
    const moveArgs: string[] = [];

    for (let i = 0; i < rowCount; i++) {
        const motorOption = <string>block.getFieldValue(`MOTOR_${i}`);
        const selectedMotorName = motorOptionToSdkName(motorOption);
        const positionInput = String(
            generator.valueToCode(block, `ANGLE_${i}`, Order.ATOMIC),
        );
        moveArgs.push(`"${selectedMotorName}"`, positionInput);
    }

    addDefinitions(generator, {
        IMPORT_RCLPY,
        IMPORT_SYS,
        IMPORT_OS,
        IMPORT_LOGGING,
        IMPORT_PIB_SDK,
        CONFIGURE_LOGGING,
        INIT_ROS,
    });

    const functionName = generator.provideFunction_(
        "move_motors",
        MOVE_MOTORS_FUNCTION(generator),
    );

    return `${functionName}(${moveArgs.join(", ")})\n`;
}

export function motor_current(
    block: Block,
    generator: typeof pythonGenerator,
): [string, Order] {
    const motorOption = <string>block.getFieldValue("MOTORNAME");
    const selectedMotorName = motorOptionToSdkName(motorOption);

    addDefinitions(generator, {
        IMPORT_OS,
        IMPORT_PIB_SDK_TELEMETRY,
    });

    const functionName = generator.provideFunction_(
        "get_motor_current_ma",
        GET_MOTOR_CURRENT_MA_FUNCTION(generator),
    );

    return [`${functionName}("${selectedMotorName}")`, Order.FUNCTION_CALL];
}

export function set_hand_position_xyz(
    block: Block,
    generator: typeof pythonGenerator,
) {
    const mode = <string>block.getFieldValue("MODE");
    if (mode !== "ABSOLUTE" && mode !== "RELATIVE") {
        throw new Error(`unexpected input-mode: ${mode}.`);
    }
    const side = readSide(block);
    const xInput = String(
        generator.valueToCode(block, "X", Order.ATOMIC) || "0",
    );
    const yInput = String(
        generator.valueToCode(block, "Y", Order.ATOMIC) || "0",
    );
    const zInput = String(
        generator.valueToCode(block, "Z", Order.ATOMIC) || "0",
    );

    addDefinitions(generator, {
        IMPORT_RCLPY,
        IMPORT_SYS,
        IMPORT_OS,
        IMPORT_LOGGING,
        IMPORT_PIB_SDK_IK,
        IMPORT_PIB_SDK_GET_HAND_POSITION_XYZ,
        IMPORT_PIB_SDK_REP103,
        CONFIGURE_LOGGING,
        INIT_ROS,
    });

    const functionName = generator.provideFunction_(
        "set_hand_position_xyz",
        SET_HAND_POSITION_XYZ_FUNCTION(generator),
    );

    return `${functionName}("${side}", "${mode}", ${xInput}, ${yInput}, ${zInput})\n`;
}

function readSide(block: Block): string {
    const side = <string>block.getFieldValue("SIDE");
    if (side !== "left" && side !== "right") {
        throw new Error(`'${side}' is not a valid value for 'SIDE'.`);
    }
    return side;
}

function getHandPositionComponent(
    block: Block,
    generator: typeof pythonGenerator,
    index: number,
): [string, Order] {
    const side = readSide(block);

    addDefinitions(generator, {
        IMPORT_PIB_SDK_GET_HAND_POSITION_XYZ,
        IMPORT_PIB_SDK_REP103,
    });

    const functionName = generator.provideFunction_(
        "get_hand_position_rep103",
        GET_HAND_POSITION_REP103_FUNCTION(generator),
    );

    return [`${functionName}("${side}")[${index}]`, Order.MEMBER];
}

export function get_hand_x(block: Block, generator: typeof pythonGenerator) {
    return getHandPositionComponent(block, generator, 0);
}

export function get_hand_y(block: Block, generator: typeof pythonGenerator) {
    return getHandPositionComponent(block, generator, 1);
}

export function get_hand_z(block: Block, generator: typeof pythonGenerator) {
    return getHandPositionComponent(block, generator, 2);
}

