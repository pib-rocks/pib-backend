import * as Blockly from "blockly";

export const playWav = Blockly.common.createBlockDefinitionsFromJsonArray([
    {
        type: "play_wav",
        message0: "play sound %1",
        args0: [
            {
                type: "field_dropdown",
                name: "WAVFILE",
                options: [
                    ["Attention", "/home/pib/wav-files/attention.wav"],
                    ["Beep", "/home/pib/wav-files/beep.wav"],
                    ["Chirp", "/home/pib/wav-files/chirp.wav"],
                    ["Confirm", "/home/pib/wav-files/confirm.wav"],
                    ["Double Beep", "/home/pib/wav-files/double_beep.wav"],
                    ["Error", "/home/pib/wav-files/error.wav"],
                    ["Glitch", "/home/pib/wav-files/glitch.wav"],
                    ["Happy", "/home/pib/wav-files/happy.wav"],
                    ["Laugh", "/home/pib/wav-files/laugh.wav"],
                    ["Notification", "/home/pib/wav-files/notification.wav"],
                    ["Processing", "/home/pib/wav-files/processing.wav"],
                    ["Sad", "/home/pib/wav-files/sad.wav"],
                    ["Scan", "/home/pib/wav-files/scan.wav"],
                    ["Shutdown", "/home/pib/wav-files/shutdown.wav"],
                    ["Sleep", "/home/pib/wav-files/sleep.wav"],
                    ["Startup", "/home/pib/wav-files/startup.wav"],
                    ["Success", "/home/pib/wav-files/success.wav"],
                    ["Surprise", "/home/pib/wav-files/surprise.wav"],
                    ["Thinking", "/home/pib/wav-files/thinking.wav"],
                    ["Warning", "/home/pib/wav-files/warning.wav"],
                    ["R2D2", "/home/pib/wav-files/R2D2.wav"],
                ],
            },
        ],
        previousStatement: null,
        nextStatement: null,
        colour: 260,
        tooltip: "plays a wav file",
        helpUrl: "",
    },
]);
