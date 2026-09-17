import * as Blockly from "blockly";

const MODEL_ID_INPUT = {
    type: "input_value",
    name: "MODEL_ID",
    check: "String",
};

export const modelBlocks = Blockly.common.createBlockDefinitionsFromJsonArray([
    {
        type: "start_model",
        message0: "start model %1 with shaves %2",
        args0: [
            MODEL_ID_INPUT,
            {
                type: "input_value",
                name: "SHAVES",
                check: "Number",
                extensions: "number_validation",
            },
        ],
        previousStatement: null,
        nextStatement: null,
        colour: 200,
        tooltip:
            "Starts the named vision model. Use 0 shaves to use the model registry default.",
        helpUrl: "",
    },
    {
        type: "stop_model",
        message0: "stop model %1",
        args0: [MODEL_ID_INPUT],
        previousStatement: null,
        nextStatement: null,
        colour: 200,
        tooltip: "Releases this program's ownership of the named vision model.",
        helpUrl: "",
    },
    {
        type: "get_detection_field",
        message0: "detection from model %1 item %2 field %3 name %4",
        args0: [
            MODEL_ID_INPUT,
            {
                type: "input_value",
                name: "INDEX",
                check: "Number",
                extensions: "number_validation",
            },
            {
                type: "field_dropdown",
                name: "FIELD",
                options: [
                    ["label", "label"],
                    ["score", "score"],
                    ["x min", "x_min"],
                    ["y min", "y_min"],
                    ["x max", "x_max"],
                    ["y max", "y_max"],
                    ["keypoint x", "keypoint_x"],
                    ["keypoint y", "keypoint_y"],
                    ["keypoint z", "keypoint_z"],
                    ["scalar value", "scalar_values"],
                ],
            },
            {
                type: "input_value",
                name: "NAME",
                check: "String",
            },
        ],
        output: null,
        colour: 200,
        tooltip:
            "Reads a field from the latest detection. Name selects a named keypoint or scalar; it is ignored for label, score, and bounding-box fields.",
        helpUrl: "",
    },
]);
