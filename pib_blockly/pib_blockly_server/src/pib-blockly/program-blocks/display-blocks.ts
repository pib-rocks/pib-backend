import * as Blockly from "blockly";

export const displayBlocks = Blockly.common.createBlockDefinitionsFromJsonArray([
  {
    type: "set_face_expression",
    message0: "Set expression %1",
    args0: [
      {
        type: "field_dropdown",
        name: "EXPRESSION",
        options: [
          ["Happy", "happy"],
          ["Sad", "sad"],
          ["Sleep", "sleep"],
          ["Surprise", "surprise"],
          ["Thinking", "thinking"],
          ["Heart eyes", "heart_eyes"],
          ["Dizzy", "dizzy"],
          ["Angry", "angry"],
          ["Laugh", "laugh"],
          ["Cry", "cry"],
        ],
      },
    ],
    previousStatement: null,
    nextStatement: null,
    colour: 180,
    tooltip: "Shows an expression on Pib's face for 15 seconds.",
    helpUrl: "",
  },
  {
    type: "show_face_text",
    message0: "Show face text %1",
    args0: [
      {
        type: "field_input",
        name: "TEXT",
        text: "Hello Pib!",
        spellcheck: false,
      },
    ],
    previousStatement: null,
    nextStatement: null,
    colour: 180,
    tooltip: "Shows short text on Pib's face. Max 40 characters.",
    helpUrl: "",
  },
]);
