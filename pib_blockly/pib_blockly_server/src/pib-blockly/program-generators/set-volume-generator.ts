import {Block} from "blockly/core/block";
import {Order, pythonGenerator} from "blockly/python";

import {CONFIGURE_LOGGING, IMPORT_LOGGING, IMPORT_RCLPY, IMPORT_SET_VOLUME, IMPORT_SYS, INIT_ROS, INIT_SET_VOLUME_CLIENT, addDefinitions} from "./util/definitions";
import {SET_VOLUME_FUNCTION} from "./util/function-declarations";

export function set_volume(block: Block, generator: typeof pythonGenerator) {
    const percent =
        generator.valueToCode(block, "PERCENT", Order.ATOMIC) || "100";

    addDefinitions(generator, {
        IMPORT_RCLPY,
        IMPORT_SYS,
        IMPORT_LOGGING,
        IMPORT_SET_VOLUME,
        CONFIGURE_LOGGING,
        INIT_ROS,
        INIT_SET_VOLUME_CLIENT,
    });

    const functionName = generator.provideFunction_(
        "set_volume",
        SET_VOLUME_FUNCTION(generator),
    );

    return `${functionName}(${percent})\n`;
}

