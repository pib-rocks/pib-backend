import {Block} from "blockly/core/block";
import {Order, pythonGenerator} from "blockly/python";
import {IMPORT_OS, IMPORT_PIB_SDK_CAMERA} from "./util/definitions";
import {
    GET_CAMERA_DEPTH_FRAME_FUNCTION,
    GET_CAMERA_DISTANCE_AT_PX_FUNCTION,
} from "./util/function-declarations";

function addCameraDefinitions(generator: typeof pythonGenerator) {
    Object.assign(generator.definitions_, {
        IMPORT_OS,
        IMPORT_PIB_SDK_CAMERA,
    });
}

export function camera_get_depth_frame(
    _block: Block,
    generator: typeof pythonGenerator,
): [string, Order] {
    addCameraDefinitions(generator);

    const functionName = generator.provideFunction_(
        "get_camera_depth_frame",
        GET_CAMERA_DEPTH_FRAME_FUNCTION(generator),
    );

    return [`${functionName}()`, Order.FUNCTION_CALL];
}

export function camera_get_distance_at_px(
    block: Block,
    generator: typeof pythonGenerator,
): [string, Order] {
    const xInput = String(generator.valueToCode(block, "X", Order.ATOMIC) || "0");
    const yInput = String(generator.valueToCode(block, "Y", Order.ATOMIC) || "0");

    addCameraDefinitions(generator);

    const functionName = generator.provideFunction_(
        "get_camera_distance_at_px",
        GET_CAMERA_DISTANCE_AT_PX_FUNCTION(generator),
    );

    return [`${functionName}(${xInput}, ${yInput})`, Order.FUNCTION_CALL];
}
