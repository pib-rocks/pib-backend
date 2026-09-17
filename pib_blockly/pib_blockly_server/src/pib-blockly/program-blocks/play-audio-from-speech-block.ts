import * as Blockly from "blockly";

export const playAudioFromSpeech =
    Blockly.common.createBlockDefinitionsFromJsonArray([
        {
            type: "play_audio_from_speech",
            message0: "say %1 in %2 with voice %3",
            args0: [
                {
                    type: "input_value",
                    name: "TEXT_INPUT",
                    check: "String",
                },
                {
                    type: "field_dropdown",
                    name: "LANGUAGE",
                    options: [
                        ["Deutsch (DE)", '"de"'],
                        ["English (EN)", '"en"'],
                        ["Auto", '"auto"'],
                    ],
                },
                {
                    type: "field_dropdown",
                    name: "VOICENAME",
                    options: [
                        ["Female 1 (F1)", '"F1"'],
                        ["Female 2 (F2)", '"F2"'],
                        ["Female 3 (F3)", '"F3"'],
                        ["Female 4 (F4)", '"F4"'],
                        ["Female 5 (F5)", '"F5"'],
                        ["Male 1 (M1)", '"M1"'],
                        ["Male 2 (M2)", '"M2"'],
                        ["Male 3 (M3)", '"M3"'],
                        ["Male 4 (M4)", '"M4"'],
                        ["Male 5 (M5)", '"M5"'],
                    ],
                },
            ],
            previousStatement: null,
            nextStatement: null,
            colour: 260,
            tooltip:
                "Speak text using local Supertonic-3 TTS with selected voice and language",
            helpUrl: "",
        },
    ]);
