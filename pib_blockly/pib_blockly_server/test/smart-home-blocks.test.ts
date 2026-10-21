import * as Blockly from "blockly";
import {Block} from "blockly/core/block";
import {pythonGenerator} from "blockly/python";
import {smartHomeBlocks} from "../src/pib-blockly/program-blocks/smart-home-blocks";
import {
    smarthome_get_state,
    smarthome_set_device,
} from "../src/pib-blockly/program-generators/smart-home-generators";

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

Blockly.common.defineBlocks(smartHomeBlocks);

const workspace = new Blockly.Workspace();
try {
    for (const blockType of ["smarthome_set_device", "smarthome_get_state"]) {
        const block = workspace.newBlock(blockType);
        assert(
            block.getField("DEVICE") !== null,
            `${blockType} must expose a DEVICE field`,
        );
    }
    const setBlock = workspace.newBlock("smarthome_set_device");
    const actionField = setBlock.getField("ACTION");
    assert(
        actionField instanceof Blockly.FieldDropdown,
        "smarthome_set_device must use a dropdown for ACTION",
    );
    const actionValues = actionField
        .getOptions(false)
        .map((option) => option[1]);
    assert(
        actionValues.includes("turn_on") && actionValues.includes("set_brightness"),
        "smarthome_set_device ACTION dropdown must include core actions",
    );
} finally {
    workspace.dispose();
}

type MockGenerator = typeof pythonGenerator & {
    definitions_: Record<string, string>;
};

function generatorWithDefinitions(): MockGenerator {
    const generator = Object.create(pythonGenerator) as MockGenerator;
    generator.definitions_ = {};
    generator.provideFunction_ = (name: string, code: string) => {
        generator.definitions_[`FN_${name}`] = code.replace(
            generator.FUNCTION_NAME_PLACEHOLDER_,
            name,
        );
        return name;
    };
    return generator;
}

function setDeviceBlock(
    device: string,
    action: string,
    value: string,
): Block {
    return {
        getFieldValue: (name: string) => {
            if (name === "DEVICE") {
                return device;
            }
            if (name === "ACTION") {
                return action;
            }
            if (name === "VALUE") {
                return value;
            }
            throw new Error(`unexpected field ${name}`);
        },
    } as unknown as Block;
}

function getStateBlock(device: string): Block {
    return {
        getFieldValue: (name: string) => {
            assertEqual(name, "DEVICE", "generator requested wrong field");
            return device;
        },
    } as unknown as Block;
}

const setGenerator = generatorWithDefinitions();
assertEqual(
    smarthome_set_device(
        setDeviceBlock("light.kitchen", "turn_on", ""),
        setGenerator,
    ),
    "smarthome_set_device('light.kitchen', 'turn_on', '')\n",
    "smarthome_set_device generated the wrong call",
);
const setPython = Object.values(setGenerator.definitions_).join("\n");
assert(
    setPython.includes("from pib_api_client import smart_home_client") &&
        setPython.includes("smart_home_client.execute_action"),
    "smarthome_set_device must use smart_home_client.execute_action",
);

const getGenerator = generatorWithDefinitions();
const [getExpr] = smarthome_get_state(
    getStateBlock("sensor.temperature"),
    getGenerator,
);
assertEqual(
    getExpr,
    "smarthome_get_state('sensor.temperature')",
    "smarthome_get_state generated the wrong expression",
);
const getPython = Object.values(getGenerator.definitions_).join("\n");
assert(
    getPython.includes("smart_home_client.get_device_state"),
    "smarthome_get_state must use smart_home_client.get_device_state",
);

console.log("smart-home block tests passed");
