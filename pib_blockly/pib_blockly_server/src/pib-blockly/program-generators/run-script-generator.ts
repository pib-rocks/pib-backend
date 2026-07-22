import {Block} from "blockly/core/block";
import {Order, pythonGenerator} from "blockly/python";
import {
    CONFIGURE_LOGGING,
    IMPORT_LOGGING,
    IMPORT_OS,
    IMPORT_PARAMIKO,
    IMPORT_SYS,
} from "./util/definitions";
import {RUN_SCRIPT_FUNCTION} from "./util/function-declarations";

export function run_script(block: Block, generator: typeof pythonGenerator) {
    // extract block-input
    const script = generator.valueToCode(block, "SCRIPT", Order.NONE) || '""';
    const host = (block as any).host_ ?? "localhost";
    const user = (block as any).user_ ?? "pib";
    const password = (block as any).password_ ?? "pib";
    const port = (block as any).port_ ?? 22;

    // add definitions to generator
    Object.assign(generator.definitions_, {
        IMPORT_SYS,
        IMPORT_OS,
        IMPORT_LOGGING,
        IMPORT_PARAMIKO,
        CONFIGURE_LOGGING,
    });

    // declare the 'run_script_over_ssh'-function
    const functionName = generator.provideFunction_(
        "run_script_over_ssh",
        RUN_SCRIPT_FUNCTION(generator),
    );

    return `${functionName}(${script}, ${generator.quote_(
        host,
    )}, ${generator.quote_(user)}, ${generator.quote_(password)}, ${Number(
        port,
    )})\n`;
}

export {pythonGenerator};
