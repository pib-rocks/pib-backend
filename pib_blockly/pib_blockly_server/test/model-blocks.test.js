const assert = require("node:assert/strict");
const fs = require("node:fs");
const ts = require("typescript");

require.extensions[".ts"] = (module, filename) => {
    const source = fs.readFileSync(filename, "utf8");
    const output = ts.transpileModule(source, {
        compilerOptions: {
            module: ts.ModuleKind.CommonJS,
            moduleResolution: ts.ModuleResolutionKind.NodeJs,
            target: ts.ScriptTarget.ES2022,
            esModuleInterop: true,
        },
        fileName: filename,
    });
    module._compile(output.outputText, filename);
};

const Blockly = require("blockly");
const {pythonGenerator} = require("blockly/python");
const {
    getModelOptions,
    modelBlocks,
} = require("../src/pib-blockly/program-blocks/model-blocks");
const {
    start_model,
    stop_model,
} = require("../src/pib-blockly/program-generators/model-generators");

Blockly.common.defineBlocks(modelBlocks);

const workspace = new Blockly.Workspace();
for (const blockType of ["start_model", "stop_model"]) {
    const block = workspace.newBlock(blockType);
    const field = block.getField("MODEL_ID");
    assert.ok(
        field instanceof Blockly.FieldDropdown,
        `${blockType} MODEL_ID must be a field_dropdown`,
    );
    for (const option of field.getOptions(false)) {
        assert.equal(option.length, 2);
        assert.equal(typeof option[0], "string");
        assert.equal(typeof option[1], "string");
    }
}
assert.deepEqual(getModelOptions(), [
    ["Loading models...", "__loading_models__"],
]);

function generatedFor(generatorFunction, modelId) {
    const definitions = {};
    const generator = Object.create(pythonGenerator);
    generator.definitions_ = definitions;
    const code = generatorFunction(
        {getFieldValue: (name) => {
            assert.equal(name, "MODEL_ID");
            return modelId;
        }},
        generator,
    );
    return {code, definitions: Object.values(definitions).join("\n")};
}

const start = generatedFor(start_model, "hand_tracking");
assert.match(start.definitions, /from pib_sdk import Models/);
assert.match(
    start.definitions,
    /os\.getenv\("ROSBRIDGE_HOST", "rosbridge-ws"\)/,
);
assert.match(
    start.code,
    /with Models\(host=rosbridge_host, port=9090\) as models:/,
);
assert.match(start.code, /models\.start_model\('hand_tracking'\)/);

const stop = generatedFor(stop_model, "object_detection");
assert.match(stop.definitions, /from pib_sdk import Models/);
assert.match(stop.code, /models\.stop_model\('object_detection'\)/);

const generatedModelsPython = [
    start.code,
    start.definitions,
    stop.code,
    stop.definitions,
].join("\n");
assert.doesNotMatch(generatedModelsPython, /rclpy/);
assert.doesNotMatch(generatedModelsPython, /StartModel|StopModel/);
assert.doesNotMatch(generatedModelsPython, /create_client|call_async/);

console.log("model block and pib-sdk generator checks passed");
