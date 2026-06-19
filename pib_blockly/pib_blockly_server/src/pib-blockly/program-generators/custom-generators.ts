import {pythonGenerator} from "blockly/python";

import * as tfButton from "./tf-button-generator";
import * as playWav from "./play-wav-generator";
import * as face_detector_blocks from "./detectors-generators";
import * as time_blocks from "./time-generators";
import * as motor_blocks from "./motor-generators";
import * as playAudioFromSpeech from "./play-audio-from-speech-generator";
import * as moveToPose from "./pose-generator";
import * as setSolidStateRelay from "./solid-state-relay-generator";
import {RESERVED_WORDS} from "./util/reserved-words";

export * from "blockly/python";

pythonGenerator.addReservedWords(RESERVED_WORDS);

const generators: typeof pythonGenerator.forBlock = {
    ...face_detector_blocks,
    ...time_blocks,
    ...motor_blocks,
    ...playAudioFromSpeech,
    ...moveToPose,
    ...setSolidStateRelay,
    ...playWav,
    ...tfButton,
};

for (const name in generators) {
    pythonGenerator.forBlock[name] = generators[name];
}

pythonGenerator.forBlock["play_audio_from_speech"] =
    generators["playAudioFromSpeechGenerator"];

pythonGenerator.forBlock["move_to_pose"] = generators["moveToPoseGenerator"];
