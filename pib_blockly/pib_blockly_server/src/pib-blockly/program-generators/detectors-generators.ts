import {Block} from "blockly/core/block";
import {pythonGenerator} from "blockly/python";
import {IMPORT_LOGGING} from "./util/definitions";
import {FACE_DETECTOR_CLASS} from "./util/function-declarations";

export function face_detector_start_stop(
    block: Block,
    generator: typeof pythonGenerator,
) {
    const dropDownSetting = block.getFieldValue("SETTING");

    Object.assign(generator.definitions_, {
        IMPORT_LOGGING,
    });

    const className = generator.provideFunction_(
        "FaceDetector",
        FACE_DETECTOR_CLASS(generator),
    );

    return dropDownSetting === "START"
        ? `fd = ${className}()\nlogging.info("Starting face detector")\n`
        : `fd.close()\nlogging.info("Closing face detector")\n`;
}

export function face_detector_running(
    block: Block,
    generator: typeof pythonGenerator,
) {
    const centerX = generator.getVariableName(
        block.getFieldValue("HORIZ_CENTER"),
    );
    const centerY = generator.getVariableName(
        block.getFieldValue("VERT_CENTER"),
    );

    return `${centerX}, ${centerY} = fd.updateDetector()\n`;
}

export {pythonGenerator};
