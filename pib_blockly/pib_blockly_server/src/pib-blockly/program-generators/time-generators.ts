import {Block} from "blockly/core/block";
import {Order, pythonGenerator} from "blockly/python";
import {IMPORT_TIME, addDefinitions} from "./util/definitions";

pythonGenerator.addReservedWords("time");

export function sleep_for_seconds(
    block: Block,
    generator: typeof pythonGenerator,
) {
    // extract block-input
    const sleepTime = block.getFieldValue("SECONDS");

    // add definitions to generator
    addDefinitions(generator, {
        IMPORT_TIME,
    });

    // generate code
    return `time.sleep(${sleepTime})\n`;
}

export function get_system_time(
    block: Block,
    generator: typeof pythonGenerator,
): [string, Order] {
    // add definitions to generator
    addDefinitions(generator, {
        IMPORT_TIME,
    });

    // generate code
    const code = "round(time.time() * 1000)";
    return [code, Order.ATOMIC];
}

