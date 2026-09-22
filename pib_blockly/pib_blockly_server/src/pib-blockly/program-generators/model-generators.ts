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
import {
    GET_DETECTION_FIELD_FUNCTION,
    START_MODEL_FUNCTION,
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
    const functionName = generator.provideFunction_(
        "stop_model_with_sdk",
        STOP_MODEL_FUNCTION(generator),
    );
    return `${functionName}(${modelIdFromDropdown(block, generator)})\n`;
}

export function get_detection_field(
    block: Block,
    generator: typeof pythonGenerator,
): [string, Order] {
    Object.assign(generator.definitions_, {
        IMPORT_RCLPY,
        IMPORT_TIME,
        IMPORT_LOGGING,
        IMPORT_SYS,
        IMPORT_DETECTION_ARRAY,
        CONFIGURE_LOGGING,
        INIT_ROS,
    });

    const functionName = generator.provideFunction_(
        "get_detection_field",
        GET_DETECTION_FIELD_FUNCTION(generator),
    );
    const modelId =
        generator.valueToCode(block, "MODEL_ID", Order.NONE) ||
        '"hand_tracking"';
    const index = generator.valueToCode(block, "INDEX", Order.NONE) || "0";
    const name = generator.valueToCode(block, "NAME", Order.NONE) || '""';
    const field = generator.quote_(String(block.getFieldValue("FIELD") || ""));

    return [
        `${functionName}(${modelId}, ${index}, ${field}, ${name})`,
        Order.FUNCTION_CALL,
    ];
}

export {pythonGenerator};
