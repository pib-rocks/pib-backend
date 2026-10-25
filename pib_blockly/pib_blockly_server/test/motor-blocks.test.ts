import * as Blockly from "blockly";
import {Block} from "blockly/core/block";
import {Order, pythonGenerator} from "blockly/python";
import {motor_blocks} from "../src/pib-blockly/program-blocks/motor-blocks";
import {move_motor} from "../src/pib-blockly/program-generators/motor-generators";

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

console.log("motor block tests passed");
