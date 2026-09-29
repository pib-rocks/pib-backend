import * as Blockly from "blockly";

export const visionBlocks =
    Blockly.common.createBlockDefinitionsFromJsonArray([
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
