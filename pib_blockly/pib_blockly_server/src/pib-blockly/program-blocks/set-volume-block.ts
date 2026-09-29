import * as Blockly from "blockly";

export const setVolume = Blockly.common.createBlockDefinitionsFromJsonArray([
    {
        type: "set_volume",
        message0: "set volume to %1 %%",
        args0: [
            {
                type: "input_value",
                name: "PERCENT",
                check: "Number",
            },
        ],
        previousStatement: null,
        nextStatement: null,
        colour: 260,
        tooltip: "Sets the output volume from 0 to 100 percent",
        helpUrl: "",
    },
]);
