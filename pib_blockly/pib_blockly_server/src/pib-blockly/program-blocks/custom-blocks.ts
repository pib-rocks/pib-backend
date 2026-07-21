import * as Blockly from "blockly";

import {tfButton} from "./tf-button-block";
import {playWav} from "./play-wav-block";
import {time_blocks} from "./time-blocks";
import {face_detector_blocks} from "./detectors-blocks";
import {motor_blocks} from "./motor-blocks";
import {playAudioFromSpeech} from "./play-audio-from-speech-block";
import {moveToPose} from "./pose-block";
import {setSolidStateRelay} from "./solid-state-relay-block";
import {displayBlocks} from "./display-blocks";
import {runScriptBlocks} from "./run-script-block";

export function customBlockDefinition() {
    Blockly.common.defineBlocks(displayBlocks);
    Blockly.common.defineBlocks(time_blocks);
    Blockly.common.defineBlocks(face_detector_blocks);
    Blockly.common.defineBlocks(motor_blocks);
    Blockly.common.defineBlocks(playAudioFromSpeech);
    Blockly.common.defineBlocks(moveToPose);
    Blockly.common.defineBlocks(setSolidStateRelay);
    Blockly.common.defineBlocks(playWav);
    Blockly.common.defineBlocks(tfButton);
    Blockly.common.defineBlocks(runScriptBlocks);

    face_detector_blocks["face_detector_running"].editable_ = false;
}
