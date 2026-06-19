import * as Blockly from "blockly";

export const face_detector_blocks =
    Blockly.common.createBlockDefinitionsFromJsonArray([
        {
            type: "face_detector_start_stop",
            message0: "Face Detector:  %1",
            args0: [
                {
                    type: "field_dropdown",
                    name: "SETTING",
                    options: [
                        ["start", "START"],
                        ["stop", "END"],
                    ],
                },
            ],
            previousStatement: null,
            nextStatement: null,
            colour: 200,
            tooltip:
                "Starts or stops the face detector, must be placed before and after 'face detector running'",
            helpUrl: "",
        },

        {
            type: "face_detector_running",
            message0:
                "Run the face detector and get the face coordiantes %1 Horiz-Center: %2  Vert-Center: %3",
            args0: [
                {
                    type: "input_dummy",
                },
                {
                    type: "field_variable",
                    name: "HORIZ_CENTER",
                    variable: "horiz_center",
                    variableTypes: ["Number"],
                    defaultType: "Number",
                },
                {
                    type: "field_variable",
                    name: "VERT_CENTER",
                    variable: "vert_center",
                    variableTypes: ["Number"],
                    defaultType: "Number",
                },
            ],
            previousStatement: null,
            nextStatement: null,
            colour: 200,
            tooltip:
                "Runs the face detector and stores the position of the bounding boxes in the variables",
            helpUrl: "",
        },
    ]);
