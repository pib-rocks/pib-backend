import {pythonGenerator} from "blockly/python";

import * as tfButton from "./tf-button-generator";
import * as playWav from "./play-wav-generator";
import * as visionGenerators from "./detectors-generators";
import * as time_blocks from "./time-generators";
import * as motor_blocks from "./motor-generators";
import * as camera_blocks from "./camera-generators";
import * as imu_blocks from "./imu-generators";
import * as playAudioFromSpeech from "./play-audio-from-speech-generator";
import * as moveToPose from "./pose-generator";
import * as setSolidStateRelay from "./solid-state-relay-generator";
import * as runScript from "./run-script-generator";
import {RESERVED_WORDS} from "./util/reserved-words";
import * as displayGenerators from "./display-generators";
import * as programLog from "./program-log-generator";
import * as setVolume from "./set-volume-generator";
import * as getSoundDirection from "./sound-direction-generator";
import * as modelBlocks from "./model-generators";

export * from "blockly/python";

pythonGenerator.addReservedWords(RESERVED_WORDS);

const generators: typeof pythonGenerator.forBlock = {
    ...visionGenerators,
    ...time_blocks,
    ...motor_blocks,
    ...camera_blocks,
    ...imu_blocks,
    ...playAudioFromSpeech,
    ...moveToPose,
    ...setSolidStateRelay,
    ...playWav,
    ...tfButton,
    ...displayGenerators,
    ...runScript,
    ...programLog,
    ...setVolume,
    ...getSoundDirection,
    ...modelBlocks,
};

for (const name in generators) {
    pythonGenerator.forBlock[name] = generators[name];
}

pythonGenerator.forBlock["play_audio_from_speech"] =
    generators["playAudioFromSpeechGenerator"];

pythonGenerator.forBlock["move_to_pose"] = generators["moveToPoseGenerator"];

pythonGenerator.forBlock["play_pose_sequence"] =
    generators["play_pose_sequence"];

pythonGenerator.forBlock["set_face_expression"] =
    generators["setFaceExpressionGenerator"];

pythonGenerator.forBlock["show_face_text"] =
    generators["showFaceTextGenerator"];

pythonGenerator.forBlock["toggle_cerebra_fullscreen"] =
    generators["toggleCerebraFullscreenGenerator"];
