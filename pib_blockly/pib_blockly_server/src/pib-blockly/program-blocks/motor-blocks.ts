import * as Blockly from "blockly";
import {MOTOR_NAME_DROPDOWN_OPTIONS} from "./motor-motor-options";

const SIDE_DROPDOWN = {
    type: "field_dropdown",
    name: "SIDE",
    options: [
        ["left", "left"],
        ["right", "right"],
    ],
};

const handPositionTooltip = (axis: string) =>
    `Returns the ${axis} coordinate of the selected hand in millimeters. X is forward, Y is to pib's own left, Z is up, in millimetres (REP-103). The value is the position the hand was last commanded to move to, not live encoder feedback. Left/right is from pib's own perspective: imagine standing in front of pib and looking at him (the same binding used across pib part numbering e.g. C35R/C35L).`;

export const motor_blocks = Blockly.common.createBlockDefinitionsFromJsonArray([
    {
        type: "move_motor",
        message0:
            "move motor(s) %1 %2 to %3 position in degrees from input %4",
        args0: [
            {
                type: "field_dropdown",
                name: "MOTORNAME",
                options: MOTOR_NAME_DROPDOWN_OPTIONS,
            },
            {
                type: "input_dummy",
            },
            {
                type: "field_dropdown",
                name: "MODE",
                options: [
                    ["absolute", "ABSOLUTE"],
                    ["relative", "RELATIVE"],
                ],
            },
            {
                type: "input_value",
                name: "POSITION",
                check: "Number",
                extensions: "number_validation",
            },
        ],
        previousStatement: null,
        nextStatement: null,
        colour: 355,
        tooltip:
            "Moves the selected motor to a target angle in degrees. Absolute mode sets the joint to that angle; relative mode adds the value to the current angle. Typical limits are about ±90° per joint (see motor configuration). Values are passed to the robot in degrees.",
        helpUrl: "",
    },
    {
        type: "motor_current",
        message0: "Motor current of %1",
        args0: [
            {
                type: "field_dropdown",
                name: "MOTORNAME",
                options: MOTOR_NAME_DROPDOWN_OPTIONS,
            },
        ],
        output: "Number",
        colour: 355,
        tooltip:
            "Returns the motor's electrical current draw in milliamps (from the servo bricklet). May be 0 or time out for a motor without a connected bricklet.",
        helpUrl: "",
    },
    {
        type: "set_hand_position_xyz",
        message0:
            "Set Hand Position at XYZ %1 %2 %3 X (mm) %4 Y (mm) %5 Z (mm) %6",
        args0: [
            {
                type: "field_dropdown",
                name: "MODE",
                options: [
                    ["absolute", "ABSOLUTE"],
                    ["relative", "RELATIVE"],
                ],
            },
            {
                type: "field_dropdown",
                name: "SIDE",
                options: [
                    ["left", "left"],
                    ["right", "right"],
                ],
            },
            {
                type: "input_dummy",
            },
            {
                type: "input_value",
                name: "X",
                check: "Number",
                align: "RIGHT",
                extensions: "number_validation",
            },
            {
                type: "input_value",
                name: "Y",
                check: "Number",
                align: "RIGHT",
                extensions: "number_validation",
            },
            {
                type: "input_value",
                name: "Z",
                check: "Number",
                align: "RIGHT",
                extensions: "number_validation",
            },
        ],
        inputsInline: false,
        previousStatement: null,
        nextStatement: null,
        colour: 355,
        tooltip:
            "Moves the selected hand to a target XYZ position in millimeters using inverse kinematics. X is forward, Y is to pib's own left, Z is up, in millimetres (REP-103). Absolute mode sets that workspace position; relative mode offsets the current commanded hand position along those axes. Left/right is from pib's own perspective: imagine standing in front of pib and looking at him (the same binding used across pib part numbering e.g. C35R/C35L).",
        helpUrl: "",
    },
    {
        type: "get_hand_x",
        message0: "Get hand position X (mm) of %1",
        args0: [SIDE_DROPDOWN],
        output: "Number",
        colour: 355,
        tooltip: handPositionTooltip("X"),
        helpUrl: "",
    },
    {
        type: "get_hand_y",
        message0: "Get hand position Y (mm) of %1",
        args0: [SIDE_DROPDOWN],
        output: "Number",
        colour: 355,
        tooltip: handPositionTooltip("Y"),
        helpUrl: "",
    },
    {
        type: "get_hand_z",
        message0: "Get hand position Z (mm) of %1",
        args0: [SIDE_DROPDOWN],
        output: "Number",
        colour: 355,
        tooltip: handPositionTooltip("Z"),
        helpUrl: "",
    },
]);
