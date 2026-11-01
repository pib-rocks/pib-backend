import {Block} from "blockly/core/block";
import {Order, pythonGenerator} from "blockly/python";
import {
    CONFIGURE_LOGGING,
    IMPORT_LOGGING,
    IMPORT_OS,
    IMPORT_PIB_SDK,
    IMPORT_PIB_SDK_BACKEND,
    IMPORT_PIB_SDK_PLAY_POSE_SEQUENCE_TIMED,
    IMPORT_PIB_SDK_POSE_CONTROL,
    IMPORT_PIB_SDK_POSES,
    IMPORT_PIB_SDK_TELEMETRY,
    IMPORT_POSE_CLIENT,
    IMPORT_RCLPY,
    IMPORT_SYS,
    IMPORT_TIME,
    IMPORT_URLPARSE,
    INIT_PIB_SDK_POSE_BACKEND,
    INIT_ROS,
    addDefinitions,
} from "./util/definitions";
import {
    APPLY_POSE_FUNCTION,
    CENTIDEGREES_TO_DEGREES_FUNCTION,
    PLAY_POSE_SEQUENCE_TIMED_FUNCTION,
    SAVE_CURRENT_POSE_FUNCTION,
} from "./util/function-declarations";

const DEFAULT_SAVED_POSE_NAME = "pose name";
const DEFAULT_STEP_DURATION_SECONDS = 1.5;

type PlayPoseSequenceGeneratorBlock = Block & {
    getPoseSequenceCount?: () => number;
    itemCount_?: number;
};

function quotePythonString(value: string): string {
    return `'${value
        .replaceAll("\\", "\\\\")
        .replaceAll("'", String.raw`\'`)}'`;
}

function addPoseSdkDefinitions(generator: typeof pythonGenerator): void {
    addDefinitions(generator, {
        IMPORT_OS,
        IMPORT_URLPARSE,
        IMPORT_PIB_SDK_BACKEND,
        IMPORT_PIB_SDK_POSES,
        INIT_PIB_SDK_POSE_BACKEND,
    });
}

function savedPoseNameExpression(
    block: Block,
    generator: typeof pythonGenerator,
): string {
    if (typeof generator.valueToCode === "function") {
        const connected = generator
            .valueToCode(block, "NAME", Order.NONE)
            .trim();
        if (connected) {
            return connected;
        }
    }
    return JSON.stringify(DEFAULT_SAVED_POSE_NAME);
}

function emitSaveCurrentPoseCall(
    block: Block,
    generator: typeof pythonGenerator,
): string {
    const nameExpression = savedPoseNameExpression(block, generator);

    addPoseSdkDefinitions(generator);
    addDefinitions(generator, {
        IMPORT_PIB_SDK_POSE_CONTROL,
        IMPORT_PIB_SDK_TELEMETRY,
    });

    const saveFunctionName = generator.provideFunction_(
        "save_current_pose_with_all_motors",
        SAVE_CURRENT_POSE_FUNCTION(generator),
    );
    return `${saveFunctionName}(${nameExpression})\n`;
}

function poseSequenceRowCount(block: PlayPoseSequenceGeneratorBlock): number {
    if (typeof block.getPoseSequenceCount === "function") {
        return block.getPoseSequenceCount();
    }
    if (typeof block.itemCount_ === "number") {
        return block.itemCount_;
    }
    if (typeof block.getInput === "function") {
        let count = 0;
        while (block.getInput(`STEP_${count}`)) {
            count++;
        }
        return count;
    }
    return 0;
}

function extractDynamicPoseSequenceLiteral(
    block: PlayPoseSequenceGeneratorBlock,
): string | null {
    const count = poseSequenceRowCount(block);
    if (count <= 0 || typeof block.getFieldValue !== "function") {
        return null;
    }
    const tuples: string[] = [];
    for (let i = 0; i < count; i++) {
        const poseIdOrName = String(
            block.getFieldValue(`POSE_${i}`) ?? "",
        ).trim();
        if (!poseIdOrName || poseIdOrName === "LOADING") {
            continue;
        }
        const rawDuration = Number(block.getFieldValue(`DURATION_${i}`));
        const duration =
            Number.isFinite(rawDuration) && rawDuration >= 0.1
                ? rawDuration
                : DEFAULT_STEP_DURATION_SECONDS;
        tuples.push(`(${JSON.stringify(poseIdOrName)}, ${duration})`);
    }
    return `[${tuples.join(", ")}]`;
}

export function moveToPoseGenerator(
    block: Block,
    generator: typeof pythonGenerator,
) {
    // extract block-input
    const poseInput = block.getFieldValue("POSE");

    // add definitions to generator
    addDefinitions(generator, {
        IMPORT_RCLPY,
        IMPORT_SYS,
        IMPORT_OS,
        IMPORT_LOGGING,
        IMPORT_PIB_SDK,
        IMPORT_POSE_CLIENT,
        CONFIGURE_LOGGING,
        INIT_ROS,
    });

    // declare the helper functions
    generator.provideFunction_(
        "centidegrees_to_degrees",
        CENTIDEGREES_TO_DEGREES_FUNCTION(generator),
    );
    const applyPoseFunctionName = generator.provideFunction_(
        "apply_pose",
        APPLY_POSE_FUNCTION(generator),
    );

    // generate code for block
    return `${applyPoseFunctionName}("${poseInput}")\n`;
}

export function save_current_pose(
    block: Block,
    generator: typeof pythonGenerator,
) {
    return emitSaveCurrentPoseCall(block, generator);
}

export function save_detection_as_pose(
    block: Block,
    generator: typeof pythonGenerator,
) {
    return emitSaveCurrentPoseCall(block, generator);
}

export function get_all_poses(
    _block: Block,
    generator: typeof pythonGenerator,
) {
    addPoseSdkDefinitions(generator);
    return ["list_poses(pose_backend)", Order.FUNCTION_CALL];
}

export function get_pose_joints(
    block: Block,
    generator: typeof pythonGenerator,
) {
    const name = String(block.getFieldValue("NAME") ?? "");
    addPoseSdkDefinitions(generator);
    return [
        `get_pose(pose_backend, name=${quotePythonString(
            name,
        )}).motor_angles_deg`,
        Order.MEMBER,
    ];
}

export function has_pose(block: Block, generator: typeof pythonGenerator) {
    const name = String(block.getFieldValue("NAME") ?? "");
    addPoseSdkDefinitions(generator);
    return [
        `any(pose.name == ${quotePythonString(
            name,
        )} for pose in list_poses(pose_backend))`,
        Order.FUNCTION_CALL,
    ];
}

export function pose_count(_block: Block, generator: typeof pythonGenerator) {
    addPoseSdkDefinitions(generator);
    return ["len(list_poses(pose_backend))", Order.FUNCTION_CALL];
}

function resolveSequenceArgument(
    block: Block,
    generator: typeof pythonGenerator,
): string {
    const dynamicLiteral = extractDynamicPoseSequenceLiteral(
        block as PlayPoseSequenceGeneratorBlock,
    );
    if (dynamicLiteral !== null) {
        return dynamicLiteral;
    }
    if (typeof generator.valueToCode === "function") {
        return (
            generator.valueToCode(block, "SEQUENCE", Order.NONE).trim() || "[]"
        );
    }
    return "[]";
}

export function play_pose_sequence(
    block: Block,
    generator: typeof pythonGenerator,
) {
    const sequenceCode = resolveSequenceArgument(block, generator);

    addPoseSdkDefinitions(generator);
    addDefinitions(generator, {
        IMPORT_PIB_SDK,
        IMPORT_PIB_SDK_PLAY_POSE_SEQUENCE_TIMED,
        IMPORT_PIB_SDK_TELEMETRY,
        IMPORT_TIME,
    });

    const playFunctionName = generator.provideFunction_(
        "_pib_play_pose_sequence_timed",
        PLAY_POSE_SEQUENCE_TIMED_FUNCTION(generator),
    );
    return `${playFunctionName}(${sequenceCode})\n`;
}
