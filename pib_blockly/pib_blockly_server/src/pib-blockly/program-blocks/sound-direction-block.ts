import * as Blockly from "blockly";

export const getSoundDirection =
    Blockly.common.createBlockDefinitionsFromJsonArray([
        {
            type: "get_sound_direction",
            message0: "get sound direction",
            output: "Number",
            colour: 260,
            tooltip:
                "Returns the sound-source direction in degrees. 0 degrees is directly in front of the head; the sign distinguishes right and left.",
            helpUrl: "",
        },
    ]);
