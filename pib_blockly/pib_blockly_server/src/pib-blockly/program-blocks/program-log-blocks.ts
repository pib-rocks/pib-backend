import * as Blockly from "blockly";

export const programLogBlocks =
    Blockly.common.createBlockDefinitionsFromJsonArray([
        {
            type: "program_log",
            message0: "log %1 %2",
            args0: [
                {
                    type: "field_dropdown",
                    name: "LEVEL",
                    options: [
                        ["INFO", "INFO"],
                        ["WARNING", "WARNING"],
                        ["ERROR", "ERROR"],
                        ["DEBUG", "DEBUG"],
                    ],
                },
                {
                    type: "input_value",
                    name: "TEXT",
                    check: "String",
                },
            ],
            inputsInline: true,
            previousStatement: null,
            nextStatement: null,
            colour: 160,
            tooltip:
                "Appends a timestamped line to this program's persistent log file.",
            helpUrl: "",
        },
        {
            type: "program_reset_log",
            message0: "reset log",
            previousStatement: null,
            nextStatement: null,
            colour: 160,
            tooltip: "Clears this program's persistent log file.",
            helpUrl: "",
        },
    ]);
