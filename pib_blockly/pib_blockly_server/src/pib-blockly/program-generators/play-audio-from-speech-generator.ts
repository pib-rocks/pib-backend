import {Block} from "blockly/core/block";

import {Order, pythonGenerator} from "blockly/python";
import {
    CONFIGURE_LOGGING,
    IMPORT_LOGGING,
    IMPORT_PLAY_AUDIO_FROM_SPREECH,
    IMPORT_RCLPY,
    IMPORT_SYS,
    INIT_PLAY_AUDIO_FROM_SPEECH_CLIENT,
    INIT_ROS,
} from "./util/definitions";
import {PLAY_AUDIO_FROM_SPEECH_FUNCTION} from "./util/function-declarations";

export function playAudioFromSpeechGenerator(
    block: Block,
    generator: typeof pythonGenerator,
) {
    // extract block-input
    const voiceName = <string>block.getFieldValue("VOICENAME");
    const textInput = generator.valueToCode(block, "TEXT_INPUT", Order.ATOMIC);

    // add definitions to generator
    Object.assign(generator.definitions_, {
        IMPORT_RCLPY,
        IMPORT_SYS,
        IMPORT_LOGGING,
        IMPORT_PLAY_AUDIO_FROM_SPREECH,
        CONFIGURE_LOGGING,
        INIT_ROS,
        INIT_PLAY_AUDIO_FROM_SPEECH_CLIENT,
    });

    // declare the 'play_audio_from_speech'-function
    const functionName = generator.provideFunction_(
        "play_audio_from_speech",
        PLAY_AUDIO_FROM_SPEECH_FUNCTION(generator),
    );

    return `${functionName}(${textInput}, ${voiceName})\n`;
}

export {pythonGenerator};
