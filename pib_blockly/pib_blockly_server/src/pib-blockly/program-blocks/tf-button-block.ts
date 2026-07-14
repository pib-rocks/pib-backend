import * as Blockly from "blockly";

export const tfButton = Blockly.common.createBlockDefinitionsFromJsonArray([
  {
    type: "tf_button_set_color",
    message0: "button %1 set color %2",
    args0: [
      {
        type: "field_dropdown",
        name: "BUTTON_ID",
        options: [
          ["1", "1"],
          ["2", "2"],
          ["3", "3"],
        ],
      },
      {
        type: "field_colour",
        name: "COLOR",
        colour: "#00ff",
      },
    ],
    previousStatement: null,
    nextStatement: null,
    colour: 20,
    tooltip: "Setzt die RGB-LED-Farbe des Tinkerforge Buttons.",
    helpUrl: "",
  },
  {
    type: "tf_button_taster_to_variable",
    message0: "button %1 as pushbutton |  color %2 write state to %3",
    args0: [
      {
        type: "field_dropdown",
        name: "BUTTON_ID",
        options: [
          ["1", "1"],
          ["2", "2"],
          ["3", "3"],
        ],
      },
      {
        type: "field_colour",
        name: "COLOR",
        colour: "#00ff00",
      },
      {
        type: "field_variable",
        name: "VAR",
        variable: "button_state",
      },
    ],
    previousStatement: null,
    nextStatement: null,
    colour: 20,
    tooltip: "Setzt die LED-Farbe und schreibt 1 in die Variable, wenn der Button gedrückt ist, sonst 0.",
    helpUrl: "",
  },
  {
    type: "tf_button_switch_to_variable",
    message0: "button %1 as switch | color %2 write state to %3",
    args0: [
      {
        type: "field_dropdown",
        name: "BUTTON_ID",
        options: [
          ["1", "1"],
          ["2", "2"],
          ["3", "3"],
        ],
      },
      {
        type: "field_colour",
        name: "COLOR",
        colour: "#ff0000",
      },
      {
        type: "field_variable",
        name: "VAR",
        variable: "button_state",
      },
    ],
    previousStatement: null,
    nextStatement: null,
    colour: 20,
    tooltip: "Setzt die LED-Farbe und schreibt den Schalterzustand als 1 oder 0 in die Variable.",
    helpUrl: "",
  },
]);
