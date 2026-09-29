import {Block} from "blockly/core/block";
import {Order, pythonGenerator} from "blockly/python";
import {
    CONFIGURE_LOGGING,
    IMPORT_INT32,
    IMPORT_LOGGING,
    IMPORT_RCLPY,
    IMPORT_SYS,
    IMPORT_TIME,
    INIT_ROS,
} from "./util/definitions";
import {GET_SOUND_DIRECTION_FUNCTION} from "./util/function-declarations";

export function get_sound_direction(
    _block: Block,
    generator: typeof pythonGenerator,
): [string, Order] {
    Object.assign(generator.definitions_, {
        CONFIGURE_LOGGING,
        IMPORT_LOGGING,
        IMPORT_SYS,
        IMPORT_RCLPY,
        IMPORT_TIME,
        IMPORT_INT32,
        INIT_ROS,
    });

    const functionName = generator.provideFunction_(
        "get_sound_direction",
        GET_SOUND_DIRECTION_FUNCTION(generator),
    );

    return [`${functionName}()`, Order.FUNCTION_CALL];
}

export {pythonGenerator};
