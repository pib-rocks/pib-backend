import * as Blockly from "blockly";

export const time_blocks = Blockly.common.createBlockDefinitionsFromJsonArray([
    {
        type: "sleep_for_seconds",
        message0: "Sleep for  %1 seconds",
        args0: [
            {
                type: "field_number",
                name: "SECONDS",
                value: 0.1,
                min: 0.001,
                max: 9999,
                precision: 0.001,
            },
        ],
        inputsInline: true,
        previousStatement: null,
        nextStatement: null,
        colour: 60,
        tooltip:
            "Sleeps for specified time. Accepts numbers with a maximum of three decimal places",
        helpUrl: "",
    },
    {
        type: "get_system_time",
        message0: "get system time",
        output: "Number",
        colour: 60,
        tooltip: "Get the system time in milliseconds from 01. 01. 1970",
        helpUrl: "",
    },
]);
