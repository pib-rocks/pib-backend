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

        {
            type: "vision_object_detected",
            message0: "does pib recognize %1 write 1 if yes, else 0 to %2",
            args0: [
                {
                    type: "field_input",
                    name: "OBJECT_NAME",
                    text: "banana",
                },
                {
                    type: "field_variable",
                    name: "RESULT",
                    variable: "object_detected",
                    variableTypes: ["Number"],
                    defaultType: "Number",
                },
            ],
            previousStatement: null,
            nextStatement: null,
            colour: 200,
            tooltip:
                "Uses the camera and Vision AI to check whether the entered object is visible. Result: 1 or 0.",
            helpUrl: "",
        },

        {
            type: "vision_object_count",
            message0:
                "does pib recognize at least %1 objects matching %2 write 1 if yes, else 0 to %3",
            args0: [
                {
                    type: "field_number",
                    name: "COUNT",
                    value: 1,
                    min: 1,
                    precision: 1,
                },
                {
                    type: "field_input",
                    name: "OBJECT_NAME",
                    text: "apple",
                },
                {
                    type: "field_variable",
                    name: "RESULT",
                    variable: "object_count_detected",
                    variableTypes: ["Number"],
                    defaultType: "Number",
                },
            ],
            previousStatement: null,
            nextStatement: null,
            colour: 200,
            tooltip:
                "Checks whether at least the selected number of objects is visible. Result: 1 or 0.",
            helpUrl: "",
        },

        {
            type: "vision_objects_different",
            message0:
                "are %1 and %2 different objects write 1 if yes, else 0 to %3",
            args0: [
                {
                    type: "field_input",
                    name: "OBJECT_A",
                    text: "apple",
                },
                {
                    type: "field_input",
                    name: "OBJECT_B",
                    text: "banana",
                },
                {
                    type: "field_variable",
                    name: "RESULT",
                    variable: "objects_different",
                    variableTypes: ["Number"],
                    defaultType: "Number",
                },
            ],
            previousStatement: null,
            nextStatement: null,
            colour: 200,
            tooltip:
                "Checks whether two visible objects are different. Example: apple and banana returns 1, two apples returns 0.",
            helpUrl: "",
        },

        {
            type: "vision_describe_image",
            message0: "describe image in %1 write text to %2",
            args0: [
                {
                    type: "field_dropdown",
                    name: "LANGUAGE",
                    options: [
                        ["German", "de"],
                        ["English", "en"],
                    ],
                },
                {
                    type: "field_variable",
                    name: "RESULT",
                    variable: "image_description",
                    variableTypes: ["String"],
                    defaultType: "String",
                },
            ],
            previousStatement: null,
            nextStatement: null,
            colour: 200,
            tooltip:
                "Describes the current camera image with Vision AI and writes the text to a variable.",
            helpUrl: "",
        },
    ]);
