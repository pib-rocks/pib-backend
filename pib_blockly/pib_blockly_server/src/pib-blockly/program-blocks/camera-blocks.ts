import * as Blockly from "blockly";

export const camera_blocks = Blockly.common.createBlockDefinitionsFromJsonArray([
    {
        type: "camera_get_depth_frame",
        message0: "get camera depth frame",
        output: null,
        colour: 200,
        tooltip:
            "Returns the camera depth image as millimetre distances (uint16, height x width). May be empty if no depth data is available.",
        helpUrl: "",
    },
    {
        type: "camera_get_distance_at_px",
        message0: "depth at x %1 y %2",
        args0: [
            {
                type: "input_value",
                name: "X",
                check: "Number",
                extensions: "number_validation",
            },
            {
                type: "input_value",
                name: "Y",
                check: "Number",
                extensions: "number_validation",
            },
        ],
        inputsInline: true,
        output: "Number",
        colour: 200,
        tooltip:
            "Returns the depth distance in millimetres at pixel (x, y). Returns 0 if the pixel is invalid or no depth data is available.",
        helpUrl: "",
    },
]);
