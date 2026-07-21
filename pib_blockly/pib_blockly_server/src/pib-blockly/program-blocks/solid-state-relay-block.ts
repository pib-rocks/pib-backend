import * as Blockly from "blockly";

export const setSolidStateRelay =
    Blockly.common.createBlockDefinitionsFromJsonArray([
        {
            type: "set_solid_state_relay",
            message0: "set Solid-State Relay:  %1",
            args0: [
                {
                    type: "field_dropdown",
                    name: "STATUS",
                    options: [
                        ["ON", "ON"],
                        ["OFF", "OFF"],
                    ],
                },
            ],
            previousStatement: null,
            nextStatement: null,
            colour: 355,
            tooltip: "Turns the Solid-State Relay on or off.",
            helpUrl: "",
        },
        {
            type: "get_solid_state_relay",
            message0: "Solid-State Relay is on",
            output: "Boolean",
            colour: 355,
            tooltip:
                "Returns whether the Solid-State Relay is currently turned on.",
            helpUrl: "",
        },
    ]);
