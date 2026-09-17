import {Block} from "blockly/core/block";
import {Order, pythonGenerator} from "blockly/python";
import {
    CONFIGURE_LOGGING,
    IMPORT_ATEXIT,
    IMPORT_DETECTION_ARRAY,
    IMPORT_LOGGING,
    IMPORT_MODEL_SERVICES,
    IMPORT_OS,
    IMPORT_RCLPY,
    IMPORT_SIGNAL,
    IMPORT_SYS,
    IMPORT_TIME,
    INIT_ROS,
} from "./util/definitions";
import {
    GET_DETECTION_FIELD_FUNCTION,
    MODEL_MANAGER_CLASS,
} from "./util/function-declarations";

function ensureModelManager(generator: typeof pythonGenerator) {
    Object.assign(generator.definitions_, {
        IMPORT_RCLPY,
        IMPORT_OS,
        IMPORT_LOGGING,
        IMPORT_ATEXIT,
        IMPORT_SIGNAL,
        IMPORT_SYS,
        IMPORT_MODEL_SERVICES,
        CONFIGURE_LOGGING,
        INIT_ROS,
    });

    const className = generator.provideFunction_(
        "BlocklyModelManager",
        MODEL_MANAGER_CLASS(generator),
    );
    generator.definitions_["INIT_BLOCKLY_MODEL_MANAGER"] =
        `_blockly_model_manager = ${className}(` +
        `node, f"blockly-{os.getpid()}")`;
}

export function start_model(block: Block, generator: typeof pythonGenerator) {
    ensureModelManager(generator);
    const modelId =
        generator.valueToCode(block, "MODEL_ID", Order.NONE) ||
        '"hand_tracking"';
    const shaves = generator.valueToCode(block, "SHAVES", Order.NONE) || "0";
    return `_blockly_model_manager.start(${modelId}, ${shaves})\n`;
}

export function stop_model(block: Block, generator: typeof pythonGenerator) {
    ensureModelManager(generator);
    const modelId =
        generator.valueToCode(block, "MODEL_ID", Order.NONE) ||
        '"hand_tracking"';
    return `_blockly_model_manager.stop(${modelId})\n`;
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
