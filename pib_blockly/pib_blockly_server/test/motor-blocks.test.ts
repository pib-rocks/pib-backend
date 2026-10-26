import * as Blockly from "blockly";
import {Block} from "blockly/core/block";
import {Order, pythonGenerator} from "blockly/python";
import {
    MOTOR_CATEGORY_TOOLBOX_BLOCK_TYPES,
} from "../src/pib-blockly/program-blocks/motor-category-toolbox";
import {motor_blocks} from "../src/pib-blockly/program-blocks/motor-blocks";
import {moveMotorsBlocks} from "../src/pib-blockly/program-blocks/move-motors-block";
import {move_motor, move_motors} from "../src/pib-blockly/program-generators/motor-generators";

function assert(condition: unknown, message: string): asserts condition {
    if (!condition) {
        throw new Error(message);
    }
}

function assertEqual(actual: unknown, expected: unknown, message: string): void {
    assert(
        actual === expected,
        `${message}: expected ${String(expected)}, received ${String(actual)}`,
    );
}

Blockly.common.defineBlocks(motor_blocks);
Blockly.common.defineBlocks(moveMotorsBlocks);

const workspace = new Blockly.Workspace();
Blockly.Events.disable();
try {
    const moveBlock = workspace.newBlock("move_motor");
    const blockText = moveBlock
        .inputList.flatMap((input) => input.fieldRow)
        .map((field) => field.getText())
        .join(" ");
    assert(
        blockText.toLowerCase().includes("degree"),
        "move_motor block label must name degrees",
    );
    const tooltip = String(moveBlock.tooltip ?? "");
    assert(
        tooltip.toLowerCase().includes("degree"),
        "move_motor tooltip must name degrees",
    );
    assert(
        tooltip.toLowerCase().includes("relative"),
        "move_motor tooltip must explain relative mode",
    );
} finally {
    workspace.dispose();
    Blockly.Events.enable();
}

assert(
    MOTOR_CATEGORY_TOOLBOX_BLOCK_TYPES.includes("move_motors"),
    "motor category toolbox must list move_motors after move_motor",
);
assertEqual(
    MOTOR_CATEGORY_TOOLBOX_BLOCK_TYPES.indexOf("move_motors"),
    MOTOR_CATEGORY_TOOLBOX_BLOCK_TYPES.indexOf("move_motor") + 1,
    "move_motors must appear directly after move_motor in the motor palette",
);

const moveMotorsWorkspace = new Blockly.Workspace();
Blockly.Events.disable();
try {
    const moveMotorsBlock = moveMotorsWorkspace.newBlock("move_motors");
    const moveMotorsText = moveMotorsBlock
        .inputList.flatMap((input) => input.fieldRow)
        .map((field) => field.getText())
        .join(" ");
    assert(
        moveMotorsText.toLowerCase().includes("degree"),
        "move_motors block label must name degrees",
    );
    const moveMotorsTooltip = String(moveMotorsBlock.tooltip ?? "");
    assert(
        moveMotorsTooltip.toLowerCase().includes("degree"),
        "move_motors tooltip must name degrees",
    );
} finally {
    moveMotorsWorkspace.dispose();
    Blockly.Events.enable();
}

type MockGenerator = typeof pythonGenerator & {
    definitions_: Record<string, string>;
};

function generatorWithDefinitions(positionCode: string): MockGenerator {
    const generator = Object.create(pythonGenerator) as MockGenerator;
    generator.definitions_ = {};
    generator.provideFunction_ = (name: string, code: string) => {
        generator.definitions_[`FN_${name}`] = code.replace(
            generator.FUNCTION_NAME_PLACEHOLDER_,
            name,
        );
        return name;
    };
    generator.valueToCode = (_block: Block, name: string, order: Order) => {
        assertEqual(name, "POSITION", "generator requested wrong value input");
        assertEqual(order, Order.ATOMIC, "POSITION must use atomic order");
        return positionCode;
    };
    return generator;
}

function moveMotorBlock(mode: "ABSOLUTE" | "RELATIVE"): Block {
    return {
        getFieldValue: (name: string) => {
            if (name === "MOTORNAME") {
                return "ELBOW_LEFT";
            }
            if (name === "MODE") {
                return mode;
            }
            throw new Error(`unexpected field ${name}`);
        },
    } as unknown as Block;
}

const degreeInput = "45.5";
const absoluteGenerator = generatorWithDefinitions(degreeInput);
const absoluteCode = move_motor(moveMotorBlock("ABSOLUTE"), absoluteGenerator);
assertEqual(
    absoluteCode,
    `apply_joint_trajectory("elbow_left", ${degreeInput})\n`,
    "move_motor absolute must pass Blockly degrees through unchanged",
);
assert(
    !absoluteCode.includes("* 100") && !absoluteCode.includes("/ 100"),
    "move_motor absolute generated call must not scale angles",
);
const absoluteDefs = Object.values(absoluteGenerator.definitions_).join("\n");
assert(
    absoluteDefs.includes("pib_sdk.Write") &&
        absoluteDefs.includes(".move(motor_name, position)"),
    "apply_joint_trajectory must call pib_sdk.Write.move with degrees",
);

const relativeGenerator = generatorWithDefinitions("10");
const relativeCode = move_motor(moveMotorBlock("RELATIVE"), relativeGenerator);
assert(
    relativeCode.includes('get_joint_position(\'elbow_left\') + 10'),
    "move_motor relative must add degree offset to current position in degrees",
);
const relativeDefs = Object.values(relativeGenerator.definitions_).join("\n");
assert(
    relativeDefs.includes("def centidegrees_to_degrees") &&
        relativeDefs.includes("return centidegrees_to_degrees(response.position)"),
    "relative move_motor must convert ROS centidegrees only in get_joint_position helper",
);

const moveMotorsGenerator = generatorWithDefinitions("30");
moveMotorsGenerator.valueToCode = (_block: Block, name: string, order: Order) => {
    assertEqual(order, Order.ATOMIC, "ANGLE inputs must use atomic order");
    if (name === "ANGLE_0") {
        return "30";
    }
    if (name === "ANGLE_1") {
        return "15";
    }
    throw new Error(`unexpected input ${name}`);
};
const moveMotorsCode = move_motors(
    {
        getMotorRowCount: () => 2,
        getFieldValue: (field: string) => {
            if (field === "MOTOR_0") {
                return "ELBOW_LEFT";
            }
            if (field === "MOTOR_1") {
                return "WRIST_LEFT";
            }
            throw new Error(`unexpected field ${field}`);
        },
    } as unknown as Block,
    moveMotorsGenerator,
);
assertEqual(
    moveMotorsCode,
    'move_motors("elbow_left", 30, "wrist_left", 15)\n',
    "move_motors must emit one vector move call in degrees",
);
const moveMotorsDefs = Object.values(moveMotorsGenerator.definitions_).join("\n");
assert(
    moveMotorsDefs.includes("write.move(*args)"),
    "move_motors helper must call pib_sdk.Write.move once with all motor pairs",
);
assert(
    !moveMotorsCode.includes("apply_joint_trajectory"),
    "move_motors must not use per-motor apply_joint_trajectory",
);

const handWorkspace = new Blockly.Workspace();
Blockly.Events.disable();
try {
    const setHandBlock = handWorkspace.newBlock("set_hand_position_xyz");
    const setTooltip = String(setHandBlock.tooltip ?? "").toLowerCase();
    assert(
        setTooltip.includes("rep-103") &&
            setTooltip.includes("forward") &&
            setTooltip.includes("left") &&
            setTooltip.includes("up"),
        "set_hand_position_xyz tooltip must describe REP-103 forward/left/up",
    );

    for (const blockType of ["get_hand_x", "get_hand_y", "get_hand_z"]) {
        const getBlock = handWorkspace.newBlock(blockType);
        const tooltip = String(getBlock.tooltip ?? "").toLowerCase();
        assert(
            tooltip.includes("rep-103") &&
                tooltip.includes("forward") &&
                tooltip.includes("left") &&
                tooltip.includes("up"),
            `${blockType} tooltip must describe REP-103 forward/left/up`,
        );
    }
} finally {
    handWorkspace.dispose();
    Blockly.Events.enable();
}

console.log("motor block tests passed");
