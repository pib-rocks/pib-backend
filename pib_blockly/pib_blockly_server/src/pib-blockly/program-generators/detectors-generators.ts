import {Block} from "blockly/core/block";
import {pythonGenerator} from "blockly/python";
import {IMPORT_LOGGING, IMPORT_VISION_PROMPT} from "./util/definitions";
import {VISION_HELPER_CLASS} from "./util/function-declarations";

function ensureVisionHelper(generator: typeof pythonGenerator): string {
    Object.assign(generator.definitions_, {
        IMPORT_LOGGING,
        IMPORT_VISION_PROMPT,
    });

    return generator.provideFunction_(
        "VisionHelper",
        VISION_HELPER_CLASS(generator),
    );
}

function pyString(value: string): string {
    return JSON.stringify(value ?? "");
}

export function vision_object_detected(
    block: Block,
    generator: typeof pythonGenerator,
) {
    const objectName = block.getFieldValue("OBJECT_NAME");
    const result = generator.getVariableName(block.getFieldValue("RESULT"));
    const className = ensureVisionHelper(generator);

    return [
        `vision = ${className}()`,
        `${result} = vision.object_detected(${pyString(objectName)})`,
        "",
    ].join("\n");
}

export function vision_object_count(
    block: Block,
    generator: typeof pythonGenerator,
) {
    const count = Number(block.getFieldValue("COUNT") ?? 1);
    const objectName = block.getFieldValue("OBJECT_NAME");
    const result = generator.getVariableName(block.getFieldValue("RESULT"));
    const className = ensureVisionHelper(generator);

    return [
        `vision = ${className}()`,
        `${result} = vision.object_count(${pyString(objectName)}, ${count})`,
        "",
    ].join("\n");
}

export function vision_objects_different(
    block: Block,
    generator: typeof pythonGenerator,
) {
    const objectA = block.getFieldValue("OBJECT_A");
    const objectB = block.getFieldValue("OBJECT_B");
    const result = generator.getVariableName(block.getFieldValue("RESULT"));
    const className = ensureVisionHelper(generator);

    return [
        `vision = ${className}()`,
        `${result} = vision.objects_different(${pyString(objectA)}, ${pyString(
            objectB,
        )})`,
        "",
    ].join("\n");
}

export function vision_describe_image(
    block: Block,
    generator: typeof pythonGenerator,
) {
    const language = block.getFieldValue("LANGUAGE");
    const result = generator.getVariableName(block.getFieldValue("RESULT"));
    const className = ensureVisionHelper(generator);

    return [
        `vision = ${className}()`,
        `${result} = vision.describe_image(${pyString(language)})`,
        "",
    ].join("\n");
}

export {pythonGenerator};
