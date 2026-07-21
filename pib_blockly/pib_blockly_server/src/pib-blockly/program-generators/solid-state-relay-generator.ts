import {Block} from "blockly/core/block";
import {pythonGenerator} from "blockly/python";
import {
    CONFIGURE_LOGGING,
    IMPORT_LOGGING,
    IMPORT_SYS,
    IMPORT_RCLPY,
    IMPORT_SET_SOLID_STATE_RELAY,
    IMPORT_SOLID_STATE_RELAY_STATE,
    INIT_ROS,
    INIT_SET_SOLID_STATE_RELAY_STATE_CLIENT,
} from "./util/definitions";
import {
    SET_SOLID_STATE_RELAY_FUNCTION,
    GET_SOLID_STATE_RELAY_FUNCTION,
} from "./util/function-declarations";

export function set_solid_state_relay(
    block: Block,
    generator: typeof pythonGenerator,
) {
    // extract block-input
    const relayStatus = <string>block.getFieldValue("STATUS");

    // add definitions to generator
    Object.assign(generator.definitions_, {
        CONFIGURE_LOGGING,
        IMPORT_LOGGING,
        IMPORT_SYS,
        IMPORT_RCLPY,
        IMPORT_SET_SOLID_STATE_RELAY,
        IMPORT_SOLID_STATE_RELAY_STATE,
        INIT_ROS,
        INIT_SET_SOLID_STATE_RELAY_STATE_CLIENT,
    });

    // declare the 'set_solid_state_relay'-function
    const functionName = generator.provideFunction_(
        "set_solid_state_relay",
        SET_SOLID_STATE_RELAY_FUNCTION(generator),
    );

    return `${functionName}("${relayStatus}")\n`;
}

export function get_solid_state_relay(
    block: Block,
    generator: typeof pythonGenerator,
) {
    // add definitions to generator
    Object.assign(generator.definitions_, {
        CONFIGURE_LOGGING,
        IMPORT_LOGGING,
        IMPORT_SYS,
        IMPORT_RCLPY,
        IMPORT_SOLID_STATE_RELAY_STATE,
        INIT_ROS,
    });

    // declare the 'get_solid_state_relay'-function
    const functionName = generator.provideFunction_(
        "get_solid_state_relay",
        GET_SOLID_STATE_RELAY_FUNCTION(generator),
    );

    return [`${functionName}()`, generator.ORDER_FUNCTION_CALL];
}

export {pythonGenerator};
