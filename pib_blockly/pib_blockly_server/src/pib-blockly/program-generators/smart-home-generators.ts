import {Block} from "blockly/core/block";
import {Order, pythonGenerator} from "blockly/python";
import {addDefinitions} from "./util/definitions";

const IMPORT_SMART_HOME_CLIENT =
    "from pib_api_client import smart_home_client";

const SMARTHOME_SET_DEVICE_FUNCTION = (generator: typeof pythonGenerator) => `
def smarthome_set_device(device, action, value_text):
    value = None
    if value_text:
        try:
            value = int(value_text)
        except ValueError:
            try:
                value = float(value_text)
            except ValueError:
                value = value_text
    ok, result = smart_home_client.execute_action(device, action, value)
    if not ok:
        raise RuntimeError(f"SmartHome action failed: {result}")
    return result
`;

const SMARTHOME_GET_STATE_FUNCTION = (generator: typeof pythonGenerator) => `
def smarthome_get_state(device):
    ok, state = smart_home_client.get_device_state(device)
    if not ok or not state:
        raise RuntimeError(f"SmartHome device not found: {device}")
    return str(state.get("state", ""))
`;

export function smarthome_set_device(
    block: Block,
    generator: typeof pythonGenerator,
) {
    const device = generator.quote_(String(block.getFieldValue("DEVICE") || ""));
    const action = generator.quote_(String(block.getFieldValue("ACTION") || ""));
    const value = generator.quote_(String(block.getFieldValue("VALUE") || ""));

    addDefinitions(generator, {
        IMPORT_SMART_HOME_CLIENT,
    });

    const functionName = generator.provideFunction_(
        "smarthome_set_device",
        SMARTHOME_SET_DEVICE_FUNCTION(generator),
    );

    return `${functionName}(${device}, ${action}, ${value})\n`;
}

export function smarthome_get_state(
    block: Block,
    generator: typeof pythonGenerator,
): [string, Order] {
    const device = generator.quote_(String(block.getFieldValue("DEVICE") || ""));

    addDefinitions(generator, {
        IMPORT_SMART_HOME_CLIENT,
    });

    const functionName = generator.provideFunction_(
        "smarthome_get_state",
        SMARTHOME_GET_STATE_FUNCTION(generator),
    );

    return [`${functionName}(${device})`, Order.FUNCTION_CALL];
}
