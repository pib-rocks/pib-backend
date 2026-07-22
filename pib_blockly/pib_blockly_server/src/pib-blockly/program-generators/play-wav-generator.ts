import {Block} from "blockly/core/block";
import {pythonGenerator} from "blockly/python";

import {
    CONFIGURE_LOGGING,
    IMPORT_LOGGING,
    IMPORT_PLAY_AUDIO_FROM_FILE,
    IMPORT_RCLPY,
    IMPORT_SYS,
    INIT_PLAY_AUDIO_FROM_FILE_CLIENT,
    INIT_ROS,
} from "./util/definitions";
import {PLAY_AUDIO_FROM_FILE_FUNCTION} from "./util/function-declarations";

export function play_wav(block: Block, generator: typeof pythonGenerator) {
    const wavFile =
        block.getFieldValue("WAVFILE") || "/home/pib/wav-files/R2D2.wav";

    Object.assign(generator.definitions_, {
        IMPORT_RCLPY,
        IMPORT_SYS,
        IMPORT_LOGGING,
        IMPORT_PLAY_AUDIO_FROM_FILE,
        CONFIGURE_LOGGING,
        INIT_ROS,
        INIT_PLAY_AUDIO_FROM_FILE_CLIENT,
    });

    const functionName = generator.provideFunction_(
        "play_audio_from_file",
        PLAY_AUDIO_FROM_FILE_FUNCTION(generator),
    );

    return `${functionName}("${wavFile}")\n`;
}

export {pythonGenerator};
