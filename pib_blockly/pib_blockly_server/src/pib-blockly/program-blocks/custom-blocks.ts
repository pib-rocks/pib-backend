import * as Blockly from "blockly";

import {tfButton} from "./tf-button-block";
import {playWav} from "./play-wav-block";
import {time_blocks} from "./time-blocks";
import {visionBlocks} from "./detectors-blocks";
import {motor_blocks} from "./motor-blocks";
import {camera_blocks} from "./camera-blocks";
import {imu_blocks} from "./imu-blocks";
import {playAudioFromSpeech} from "./play-audio-from-speech-block";
import {moveToPose, poseBlocks} from "./pose-block";
import {setSolidStateRelay} from "./solid-state-relay-block";
import {displayBlocks} from "./display-blocks";
import {runScriptBlocks} from "./run-script-block";
import {programLogBlocks} from "./program-log-blocks";
import {setVolume} from "./set-volume-block";
import {getSoundDirection} from "./sound-direction-block";
import {modelBlocks} from "./model-blocks";

export function customBlockDefinition() {
    Blockly.common.defineBlocks(displayBlocks);
    Blockly.common.defineBlocks(time_blocks);
    Blockly.common.defineBlocks(visionBlocks);
    Blockly.common.defineBlocks(motor_blocks);
    Blockly.common.defineBlocks(camera_blocks);
    Blockly.common.defineBlocks(imu_blocks);
    Blockly.common.defineBlocks(playAudioFromSpeech);
    Blockly.common.defineBlocks(moveToPose);
    Blockly.common.defineBlocks(poseBlocks);
    Blockly.common.defineBlocks(setSolidStateRelay);
    Blockly.common.defineBlocks(playWav);
    Blockly.common.defineBlocks(tfButton);
    Blockly.common.defineBlocks(runScriptBlocks);
    Blockly.common.defineBlocks(programLogBlocks);
    Blockly.common.defineBlocks(setVolume);
    Blockly.common.defineBlocks(getSoundDirection);
    Blockly.common.defineBlocks(modelBlocks);
}
