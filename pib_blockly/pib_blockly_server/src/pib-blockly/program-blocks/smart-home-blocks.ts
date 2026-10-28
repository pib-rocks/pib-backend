import * as Blockly from "blockly";

export const smartHomeBlocks =
    Blockly.common.createBlockDefinitionsFromJsonArray([
        {
            type: "smarthome_set_device",
            message0: "set SmartHome device %1 action %2 value %3",
            args0: [
                {
                    type: "field_input",
                    name: "DEVICE",
                    text: "light.living_room",
                },
                {
                    type: "field_dropdown",
                    name: "ACTION",
                    options: [
                        ["turn on", "turn_on"],
                        ["turn off", "turn_off"],
                        ["toggle", "toggle"],
                        ["set brightness (%)", "set_brightness"],
                        ["set temperature (°C)", "set_temperature"],
                        ["set cover (%)", "set_cover_position"],
                        ["lock", "lock"],
                        ["unlock", "unlock"],
                    ],
                },
                {
                    type: "field_input",
                    name: "VALUE",
                    text: "",
                },
            ],
            previousStatement: null,
            nextStatement: null,
            colour: 45,
            tooltip: "Control a SmartHome device through the pib API.",
            helpUrl: "",
        },
        {
            type: "smarthome_get_state",
            message0: "get SmartHome state of %1",
            args0: [
                {
                    type: "field_input",
                    name: "DEVICE",
                    text: "sensor.temperature",
                },
            ],
            output: "String",
            colour: 45,
            tooltip: "Read the current state of a SmartHome device.",
            helpUrl: "",
        },
    ]);
