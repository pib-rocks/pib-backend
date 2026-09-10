import {Block} from "blockly/core/block";
import {Order, pythonGenerator} from "blockly/python";
import {IMPORT_DATETIME, IMPORT_OS} from "./util/definitions";
import {
    PROGRAM_LOG_FUNCTION,
    PROGRAM_LOG_PATH_FUNCTION,
    PROGRAM_RESET_LOG_FUNCTION,
} from "./util/function-declarations";

const VALID_LEVELS = new Set(["INFO", "WARNING", "ERROR", "DEBUG"]);

function provideProgramLogPath(generator: typeof pythonGenerator) {
    Object.assign(generator.definitions_, {
        IMPORT_OS,
        IMPORT_DATETIME,
    });

    return generator.provideFunction_(
        "program_log_path",
        PROGRAM_LOG_PATH_FUNCTION(generator),
    );
}

export function program_log(block: Block, generator: typeof pythonGenerator) {
    const level = String(block.getFieldValue("LEVEL") || "INFO");
    if (!VALID_LEVELS.has(level)) {
        throw new Error(`'${level}' is not a valid value for 'LEVEL'.`);
    }

    const text = String(
        generator.valueToCode(block, "TEXT", Order.NONE) || '""',
    );
    const pathFunctionName = provideProgramLogPath(generator);
    const functionName = generator.provideFunction_(
        "program_log",
        PROGRAM_LOG_FUNCTION(generator, pathFunctionName),
    );

    return `${functionName}("${level}", ${text})\n`;
}

export function program_reset_log(
    _block: Block,
    generator: typeof pythonGenerator,
) {
    const pathFunctionName = provideProgramLogPath(generator);
    const functionName = generator.provideFunction_(
        "program_reset_log",
        PROGRAM_RESET_LOG_FUNCTION(generator, pathFunctionName),
    );

    return `${functionName}()\n`;
}

export {pythonGenerator};
