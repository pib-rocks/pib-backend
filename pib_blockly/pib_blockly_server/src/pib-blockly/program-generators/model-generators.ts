import {Block} from "blockly/core/block";
import {Order, pythonGenerator} from "blockly/python";
import {
    CONFIGURE_LOGGING,
    IMPORT_DETECTION_ARRAY,
    IMPORT_LOGGING,
    IMPORT_OS,
    IMPORT_PIB_SDK_MODELS,
    IMPORT_RCLPY,
    IMPORT_SYS,
    IMPORT_TIME,
    INIT_ROS,
} from "./util/definitions";
import {STOP_ALL_MODELS_VALUE} from "../program-blocks/model-blocks";
import {
    GET_FACE_DETECTIONS_FUNCTION,
    GET_OBJECT_DETECTIONS_FUNCTION,
    GET_QR_DETECTIONS_FUNCTION,
    GET_EMOTION_DETECTIONS_FUNCTION,
    GET_HEAD_POSE_DETECTIONS_FUNCTION,
    START_MODEL_FUNCTION,
    STOP_ALL_MODELS_FUNCTION,
    STOP_MODEL_FUNCTION,
} from "./util/function-declarations";

function modelIdFromDropdown(
    block: Block,
    generator: typeof pythonGenerator,
): string {
    return generator.quote_(
        String(block.getFieldValue("MODEL_ID") || "hand_tracking"),
    );
}

function ensureModelsSdk(generator: typeof pythonGenerator) {
    Object.assign(generator.definitions_, {
        IMPORT_OS,
        IMPORT_PIB_SDK_MODELS,
    });
}

export function start_model(block: Block, generator: typeof pythonGenerator) {
    ensureModelsSdk(generator);
    const functionName = generator.provideFunction_(
        "start_model_with_sdk",
        START_MODEL_FUNCTION(generator),
    );
    return `${functionName}(${modelIdFromDropdown(block, generator)})\n`;
}

export function stop_model(block: Block, generator: typeof pythonGenerator) {
    ensureModelsSdk(generator);
    const modelId = String(block.getFieldValue("MODEL_ID") || "hand_tracking");
    if (modelId === STOP_ALL_MODELS_VALUE) {
        const functionName = generator.provideFunction_(
            "stop_all_models_with_sdk",
            STOP_ALL_MODELS_FUNCTION(generator),
        );
        return `${functionName}()\n`;
    }
    const functionName = generator.provideFunction_(
        "stop_model_with_sdk",
        STOP_MODEL_FUNCTION(generator),
    );
    return `${functionName}(${generator.quote_(modelId)})\n`;
}

function latestDetectionsReporter(
    functionName: string,
    declaration: (generator: typeof pythonGenerator) => string,
) {
    return (
        _block: Block,
        generator: typeof pythonGenerator,
    ): [string, Order] => {
        Object.assign(generator.definitions_, {
            IMPORT_RCLPY,
            IMPORT_TIME,
            IMPORT_LOGGING,
            IMPORT_SYS,
            IMPORT_DETECTION_ARRAY,
            CONFIGURE_LOGGING,
            INIT_ROS,
        });

        const provided = generator.provideFunction_(
            functionName,
            declaration(generator),
        );

        return [`${provided}()`, Order.FUNCTION_CALL];
    };
}

export const get_face_detections = latestDetectionsReporter(
    "get_face_detections",
    GET_FACE_DETECTIONS_FUNCTION,
);

export const get_object_detections = latestDetectionsReporter(
    "get_object_detections",
    GET_OBJECT_DETECTIONS_FUNCTION,
);

export const get_qr_detections = latestDetectionsReporter(
    "get_qr_detections",
    GET_QR_DETECTIONS_FUNCTION,
);

export const get_emotion_detections = latestDetectionsReporter(
    "get_emotion_detections",
    GET_EMOTION_DETECTIONS_FUNCTION,
);

export const get_head_pose_detections = latestDetectionsReporter(
    "get_head_pose_detections",
    GET_HEAD_POSE_DETECTIONS_FUNCTION,
);

export {pythonGenerator};
