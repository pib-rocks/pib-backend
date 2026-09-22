import * as Blockly from "blockly";
import {Block} from "blockly/core/block";
import {pythonGenerator} from "blockly/python";
import {modelBlocks} from "../src/pib-blockly/program-blocks/model-blocks";
import {
    start_model,
    stop_model,
} from "../src/pib-blockly/program-generators/model-generators";

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

Blockly.common.defineBlocks(modelBlocks);

const workspace = new Blockly.Workspace();
try {
    for (const blockType of ["start_model", "stop_model"]) {
        const block = workspace.newBlock(blockType);
        const field = block.getField("MODEL_ID");
        assert(
            field instanceof Blockly.FieldDropdown,
            `${blockType} must have a field_dropdown named MODEL_ID`,
        );
        const options = field.getOptions(false);
        assert(options.length > 0, `${blockType} MODEL_ID needs options`);
        assert(
            options.every(
                (option) =>
                    option.length === 2 &&
                    typeof option[0] === "string" &&
                    typeof option[1] === "string",
            ),
            `${blockType} MODEL_ID options must be [label, model_id] pairs`,
        );
    }

    const detectionBlock = workspace.newBlock("get_detection_field");
    assert(
        detectionBlock.getInput("MODEL_ID")?.connection?.getCheck()?.includes(
            "String",
        ),
        "get_detection_field must retain its String MODEL_ID value input",
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

function modelBlock(modelId: string): Block {
    return {
        getFieldValue: (name: string) => {
            assertEqual(name, "MODEL_ID", "generator requested wrong field");
            return modelId;
        },
    } as unknown as Block;
}

const startGenerator = generatorWithDefinitions();
assertEqual(
    start_model(modelBlock("hand_tracking"), startGenerator),
    "start_model_with_sdk('hand_tracking')\n",
    "start_model generated the wrong call",
);
const startPython = Object.values(startGenerator.definitions_).join("\n");
assert(
    startPython.includes("from pib_sdk import Models") &&
        startPython.includes(
            "with Models(host=rosbridge_host, port=9090) as models:",
        ) &&
        startPython.includes("models.start_model(str(model_id))"),
    "start_model must use pib_sdk.Models",
);

const stopGenerator = generatorWithDefinitions();
assertEqual(
    stop_model(modelBlock("face_detection"), stopGenerator),
    "stop_model_with_sdk('face_detection')\n",
    "stop_model generated the wrong call",
);
const stopPython = Object.values(stopGenerator.definitions_).join("\n");
assert(
    stopPython.includes("from pib_sdk import Models") &&
        stopPython.includes("models.stop_model(str(model_id))"),
    "stop_model must use pib_sdk.Models",
);

const modelPython = `${startPython}\n${stopPython}`;
assert(
    !modelPython.includes("from datatypes.srv import StartModel") &&
        !modelPython.includes("create_client(StartModel") &&
        !modelPython.includes("create_client(StopModel") &&
        !modelPython.includes("rclpy"),
    "model lifecycle generators must not emit direct ROS service calls",
);

console.log("model block tests passed");
