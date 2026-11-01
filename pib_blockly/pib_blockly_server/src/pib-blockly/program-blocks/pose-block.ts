import * as Blockly from "blockly";

const DEFAULT_POSE_SEQUENCE_COUNT = 2;
const MIN_POSE_SEQUENCE_COUNT = 1;
const DEFAULT_POSE_DURATION_SECONDS = 1.5;
const MIN_POSE_DURATION_SECONDS = 0.1;
const MAX_POSE_DURATION_SECONDS = 60.0;

const ADD_STEP_ICON =
    "data:image/svg+xml;utf8," +
    encodeURIComponent(
        '<svg xmlns="http://www.w3.org/2000/svg" width="18" height="18" viewBox="0 0 18 18">' +
            '<rect x="1" y="1" width="16" height="16" rx="4" fill="#ffffff" fill-opacity="0.22" stroke="#ffffff" stroke-width="1.2"/>' +
            '<path d="M9 4.5v9M4.5 9h9" stroke="#ffffff" stroke-width="2" stroke-linecap="round"/>' +
            "</svg>",
    );

const REMOVE_STEP_ICON =
    "data:image/svg+xml;utf8," +
    encodeURIComponent(
        '<svg xmlns="http://www.w3.org/2000/svg" width="18" height="18" viewBox="0 0 18 18">' +
            '<rect x="1" y="1" width="16" height="16" rx="4" fill="#ffffff" fill-opacity="0.22" stroke="#ffffff" stroke-width="1.2"/>' +
            '<path d="M4.5 9h9" stroke="#ffffff" stroke-width="2" stroke-linecap="round"/>' +
            "</svg>",
    );

export interface PoseSequenceStepState {
    pose: string;
    duration: number;
}

export interface PlayPoseSequenceBlock extends Blockly.Block {
    itemCount_: number;
    updateShape_: (steps?: PoseSequenceStepState[]) => void;
    getPoseSequenceCount: () => number;
    getStepsState: () => PoseSequenceStepState[];
    insertStepAt: (index: number, pose?: string, duration?: number) => void;
    removeStepAt: (index: number) => void;
    saveExtraState: () => {
        itemCount: number;
        steps?: PoseSequenceStepState[];
    };
    loadExtraState: (state: {
        itemCount?: number;
        steps?: PoseSequenceStepState[];
    }) => void;
    decompose: (workspace: Blockly.Workspace) => Blockly.Block;
    compose: (containerBlock: Blockly.Block) => void;
    saveConnections: (containerBlock: Blockly.Block) => void;
}

interface PoseSequenceItemBlock extends Blockly.Block {
    stepState_?: PoseSequenceStepState;
}

function getAvailablePoseOptions(): [string, string][] {
    const moveToPoseBlock = Blockly.Blocks["move_to_pose"] as
        | {getPoses?: () => [string, string][]}
        | undefined;
    if (moveToPoseBlock && typeof moveToPoseBlock.getPoses === "function") {
        const options = moveToPoseBlock.getPoses();
        if (Array.isArray(options) && options.length > 0) {
            return options;
        }
    }
    return [["Loading...", "LOADING"]];
}

function defaultPoseValue(): string {
    const options = getAvailablePoseOptions();
    return options[0]?.[1] ?? "LOADING";
}

function normalizeDuration(value: unknown): number {
    const numeric = Number(value);
    if (!Number.isFinite(numeric) || numeric < MIN_POSE_DURATION_SECONDS) {
        return DEFAULT_POSE_DURATION_SECONDS;
    }
    return Math.min(MAX_POSE_DURATION_SECONDS, numeric);
}

export const moveToPose = {
    move_to_pose: {
        init(this: Blockly.Block) {
            this.appendDummyInput()
                .appendField("set Pose")
                .appendField(
                    new CustomFieldDropdown(() => getAvailablePoseOptions()),
                    "POSE",
                );
            this.setColour(355);
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setTooltip(
                "Moves all motors to the saved pose. Joint targets are applied in degrees.",
            );
            this.setHelpUrl("");
        },

        getPoses(): [string, string][] {
            // Initial placeholder before options are fetched
            return [["Loading...", "LOADING"]];
        },
    },

    play_pose_sequence_container: {
        init(this: Blockly.Block) {
            this.appendDummyInput().appendField("pose sequence");
            this.appendStatementInput("STACK");
            this.setColour(355);
            this.setTooltip(
                "Add, remove, or reorder pose steps in this sequence.",
            );
            this.contextMenu = false;
        },
    },

    play_pose_sequence_item: {
        init(this: Blockly.Block) {
            this.appendDummyInput().appendField("pose step");
            this.setPreviousStatement(true);
            this.setNextStatement(true);
            this.setColour(355);
            this.setTooltip("One pose waypoint in the timed pose sequence.");
            this.contextMenu = false;
        },
    },

    play_pose_sequence: {
        itemCount_: DEFAULT_POSE_SEQUENCE_COUNT,

        init(this: PlayPoseSequenceBlock) {
            this.itemCount_ = DEFAULT_POSE_SEQUENCE_COUNT;
            this.appendDummyInput("HEADER").appendField("play pose sequence");
            this.setColour(355);
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setMutator(
                new Blockly.icons.MutatorIcon(
                    ["play_pose_sequence_item"],
                    this as unknown as Blockly.BlockSvg,
                ),
            );
            this.setTooltip(
                "Plays a dynamic sequence of saved poses as one smooth, continuous PCHIP trajectory. Use [+] and [-] on any row or the gear icon to insert, delete, or reorder intermediate poses.",
            );
            this.setHelpUrl("");
            this.updateShape_();
        },

        getPoseSequenceCount(this: PlayPoseSequenceBlock): number {
            return this.itemCount_;
        },

        getStepsState(this: PlayPoseSequenceBlock): PoseSequenceStepState[] {
            const steps: PoseSequenceStepState[] = [];
            for (let i = 0; i < this.itemCount_; i++) {
                if (!this.getInput(`STEP_${i}`)) {
                    continue;
                }
                const poseVal = String(
                    this.getFieldValue(`POSE_${i}`) ?? defaultPoseValue(),
                );
                const durationVal = normalizeDuration(
                    this.getFieldValue(`DURATION_${i}`),
                );
                steps.push({pose: poseVal, duration: durationVal});
            }
            return steps;
        },

        insertStepAt(
            this: PlayPoseSequenceBlock,
            index: number,
            pose?: string,
            duration: number = DEFAULT_POSE_DURATION_SECONDS,
        ): void {
            const current = this.getStepsState();
            const clampedIndex = Math.max(
                0,
                Math.min(current.length, Math.trunc(index)),
            );
            const fallbackPose =
                pose ??
                current[Math.max(0, clampedIndex - 1)]?.pose ??
                defaultPoseValue();
            current.splice(clampedIndex, 0, {
                pose: fallbackPose,
                duration: normalizeDuration(duration),
            });
            this.itemCount_ = current.length;
            this.updateShape_(current);
        },

        removeStepAt(this: PlayPoseSequenceBlock, index: number): void {
            const current = this.getStepsState();
            if (current.length <= MIN_POSE_SEQUENCE_COUNT) {
                return;
            }
            const clampedIndex = Math.max(
                0,
                Math.min(current.length - 1, Math.trunc(index)),
            );
            current.splice(clampedIndex, 1);
            this.itemCount_ = Math.max(MIN_POSE_SEQUENCE_COUNT, current.length);
            this.updateShape_(current);
        },

        saveExtraState(this: PlayPoseSequenceBlock): {
            itemCount: number;
            steps?: PoseSequenceStepState[];
        } {
            return {
                itemCount: this.itemCount_,
                steps: this.getStepsState(),
            };
        },

        loadExtraState(
            this: PlayPoseSequenceBlock,
            state: {itemCount?: number; steps?: PoseSequenceStepState[]},
        ): void {
            let rawCount = DEFAULT_POSE_SEQUENCE_COUNT;
            if (typeof state?.itemCount === "number") {
                rawCount = state.itemCount;
            } else if (Array.isArray(state?.steps)) {
                rawCount = state.steps.length;
            }
            this.itemCount_ = Math.max(MIN_POSE_SEQUENCE_COUNT, rawCount);
            this.updateShape_(
                Array.isArray(state?.steps) ? state.steps : undefined,
            );
        },

        decompose(
            this: PlayPoseSequenceBlock,
            workspace: Blockly.Workspace,
        ): Blockly.Block {
            const containerBlock = workspace.newBlock(
                "play_pose_sequence_container",
            );
            (containerBlock as unknown as {initSvg?: () => void}).initSvg?.();
            let connection = containerBlock.getInput("STACK")!.connection!;
            const currentSteps = this.getStepsState();
            for (let i = 0; i < this.itemCount_; i++) {
                const itemBlock = workspace.newBlock(
                    "play_pose_sequence_item",
                ) as PoseSequenceItemBlock;
                (itemBlock as unknown as {initSvg?: () => void}).initSvg?.();
                itemBlock.stepState_ = currentSteps[i] ?? {
                    pose: defaultPoseValue(),
                    duration: DEFAULT_POSE_DURATION_SECONDS,
                };
                connection.connect(itemBlock.previousConnection!);
                connection = itemBlock.nextConnection!;
            }
            return containerBlock;
        },

        compose(
            this: PlayPoseSequenceBlock,
            containerBlock: Blockly.Block,
        ): void {
            let itemBlock = containerBlock.getInputTargetBlock(
                "STACK",
            ) as PoseSequenceItemBlock | null;
            const steps: PoseSequenceStepState[] = [];
            while (itemBlock) {
                if (!itemBlock.isInsertionMarker()) {
                    steps.push(
                        itemBlock.stepState_ ?? {
                            pose: defaultPoseValue(),
                            duration: DEFAULT_POSE_DURATION_SECONDS,
                        },
                    );
                }
                itemBlock =
                    itemBlock.nextConnection &&
                    (itemBlock.nextConnection.targetBlock() as PoseSequenceItemBlock | null);
            }
            if (steps.length < MIN_POSE_SEQUENCE_COUNT) {
                steps.push({
                    pose: defaultPoseValue(),
                    duration: DEFAULT_POSE_DURATION_SECONDS,
                });
            }
            this.itemCount_ = steps.length;
            this.updateShape_(steps);
        },

        saveConnections(
            this: PlayPoseSequenceBlock,
            containerBlock: Blockly.Block,
        ): void {
            let itemBlock = containerBlock.getInputTargetBlock(
                "STACK",
            ) as PoseSequenceItemBlock | null;
            let i = 0;
            while (itemBlock) {
                if (!itemBlock.isInsertionMarker()) {
                    if (this.getInput(`STEP_${i}`)) {
                        itemBlock.stepState_ = {
                            pose: String(
                                this.getFieldValue(`POSE_${i}`) ??
                                    defaultPoseValue(),
                            ),
                            duration: normalizeDuration(
                                this.getFieldValue(`DURATION_${i}`),
                            ),
                        };
                    }
                    i++;
                }
                itemBlock =
                    itemBlock.nextConnection &&
                    (itemBlock.nextConnection.targetBlock() as PoseSequenceItemBlock | null);
            }
        },

        updateShape_(
            this: PlayPoseSequenceBlock,
            explicitSteps?: PoseSequenceStepState[],
        ): void {
            const preservedSteps = explicitSteps ?? this.getStepsState();

            if (this.getInput("SEQUENCE")) {
                this.removeInput("SEQUENCE");
            }

            let removeIdx = 0;
            while (this.getInput(`STEP_${removeIdx}`)) {
                this.removeInput(`STEP_${removeIdx}`);
                removeIdx++;
            }

            const total = Math.max(MIN_POSE_SEQUENCE_COUNT, this.itemCount_);
            this.itemCount_ = total;

            for (let i = 0; i < total; i++) {
                const rowStep = preservedSteps[i];
                const poseDropdown = new CustomFieldDropdown(() =>
                    getAvailablePoseOptions(),
                );
                if (rowStep?.pose) {
                    poseDropdown.setValue(rowStep.pose);
                }
                const durationValue = normalizeDuration(rowStep?.duration);
                const durationField = new Blockly.FieldNumber(
                    durationValue,
                    MIN_POSE_DURATION_SECONDS,
                    MAX_POSE_DURATION_SECONDS,
                    0.05,
                );

                const rowIndex = i;
                const addButton = new Blockly.FieldImage(
                    ADD_STEP_ICON,
                    16,
                    16,
                    "Insert pose after this step",
                    () => {
                        this.insertStepAt(rowIndex + 1);
                    },
                );

                const input = this.appendDummyInput(`STEP_${i}`)
                    .appendField(`${i + 1}. Pose`)
                    .appendField(poseDropdown, `POSE_${i}`)
                    .appendField("in")
                    .appendField(durationField, `DURATION_${i}`)
                    .appendField("s")
                    .appendField(addButton, `ADD_${i}`);

                if (total > MIN_POSE_SEQUENCE_COUNT) {
                    const removeButton = new Blockly.FieldImage(
                        REMOVE_STEP_ICON,
                        16,
                        16,
                        "Remove this pose step",
                        () => {
                            this.removeStepAt(rowIndex);
                        },
                    );
                    input.appendField(removeButton, `REMOVE_${i}`);
                }
            }
        },
    },
};

export const poseBlocks = Blockly.common.createBlockDefinitionsFromJsonArray([
    {
        type: "save_current_pose",
        message0: "Save current pose as %1",
        args0: [
            {
                type: "input_value",
                name: "NAME",
                check: "String",
            },
        ],
        previousStatement: null,
        nextStatement: null,
        colour: 355,
        tooltip:
            "Records the current motor angles in degrees and saves them to the pose list. The name comes from the string or variable connected on the right.",
        helpUrl: "",
    },
    {
        type: "save_detection_as_pose",
        message0: "save current detection as pose %1",
        args0: [
            {
                type: "input_value",
                name: "NAME",
                check: "String",
            },
        ],
        previousStatement: null,
        nextStatement: null,
        colour: 355,
        tooltip:
            "Saves the robot's current motor angles in degrees through the existing pose list. The name may be a string variable.",
        helpUrl: "",
    },
    {
        type: "get_all_poses",
        message0: "get all poses",
        output: "Array",
        colour: 355,
        tooltip: "Returns a list of all saved poses.",
        helpUrl: "",
    },
    {
        type: "get_pose_joints",
        message0: "get joints of pose %1 (degrees)",
        args0: [
            {
                type: "field_input",
                name: "NAME",
                text: "pose name",
            },
        ],
        output: "Object",
        colour: 355,
        tooltip:
            "Returns the saved motor angles in degrees, keyed by motor name.",
        helpUrl: "",
    },
    {
        type: "has_pose",
        message0: "has pose %1",
        args0: [
            {
                type: "field_input",
                name: "NAME",
                text: "pose name",
            },
        ],
        output: "Boolean",
        colour: 355,
        tooltip: "Returns whether a pose with this name exists.",
        helpUrl: "",
    },
    {
        type: "pose_count",
        message0: "pose count",
        output: "Number",
        colour: 355,
        tooltip: "Returns the number of saved poses.",
        helpUrl: "",
    },
]);

export class CustomFieldDropdown extends Blockly.FieldDropdown {
    constructor(menuGenerator: any, opt_validator?: any, opt_config?: any) {
        super(menuGenerator, opt_validator, opt_config);
    }
    // Override the default validation function
    override doClassValidation_(newValue: any) {
        return newValue;
    }
}
