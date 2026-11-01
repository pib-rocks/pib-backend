import * as Blockly from "blockly";
import {
    CustomFieldDropdown,
    moveToPose,
    PlayPoseSequenceBlock,
    poseBlocks,
} from "./pose-block";

describe("pose blocks", () => {
    beforeAll(() => {
        Blockly.common.defineBlocks(moveToPose);
        Blockly.common.defineBlocks(poseBlocks);
    });

    it("initializes move_to_pose and play_pose_sequence blocks", () => {
        Blockly.Blocks["move_to_pose"].getPoses = () => [
            ["Startup/Resting", "pose-1"],
            ["Wave 1", "pose-2"],
            ["Wave 2", "pose-3"],
        ];
        const workspace = new Blockly.Workspace();
        Blockly.Events.enable();
        try {
            const single = workspace.newBlock("move_to_pose");
            expect(single.getField("POSE")).toBeTruthy();

            const seq = workspace.newBlock(
                "play_pose_sequence",
            ) as PlayPoseSequenceBlock;
            expect(seq.getPoseSequenceCount()).toBe(2);
            seq.setFieldValue("pose-1", "POSE_0");
            seq.setFieldValue(1.2, "DURATION_0");
            seq.setFieldValue("pose-3", "POSE_1");
            seq.setFieldValue(2, "DURATION_1");

            seq.insertStepAt(1, "pose-2", 1.5);
            expect(seq.getPoseSequenceCount()).toBe(3);
            expect(seq.getStepsState()).toEqual([
                {pose: "pose-1", duration: 1.2},
                {pose: "pose-2", duration: 1.5},
                {pose: "pose-3", duration: 2},
            ]);

            const addField = seq.getField("ADD_0") as Blockly.FieldImage;
            (
                addField as unknown as {
                    showEditor_?: () => void;
                }
            ).showEditor_?.();
            expect(seq.getPoseSequenceCount()).toBe(4);

            const removeField = seq.getField("REMOVE_1") as Blockly.FieldImage;
            (
                removeField as unknown as {
                    showEditor_?: () => void;
                }
            ).showEditor_?.();
            expect(seq.getPoseSequenceCount()).toBe(3);

            seq.removeStepAt(1);
            expect(seq.getPoseSequenceCount()).toBe(2);

            seq.removeStepAt(0);
            expect(seq.getPoseSequenceCount()).toBe(1);
            seq.removeStepAt(0);
            expect(seq.getPoseSequenceCount()).toBe(1);

            seq.insertStepAt(1, "pose-2", -5);
            expect(seq.getStepsState()[1].duration).toBe(1.5);

            const state = seq.saveExtraState();
            const restored = workspace.newBlock(
                "play_pose_sequence",
            ) as PlayPoseSequenceBlock;
            restored.loadExtraState(state);
            expect(restored.getStepsState()).toEqual(seq.getStepsState());

            restored.loadExtraState({
                steps: [{pose: "pose-1", duration: 1}],
            });
            expect(restored.getPoseSequenceCount()).toBe(1);
            restored.loadExtraState({});
            expect(restored.getPoseSequenceCount()).toBe(2);

            const mutatorWs = new Blockly.Workspace();
            const container = restored.decompose(mutatorWs);
            restored.saveConnections(container);
            restored.compose(container);
            expect(restored.getPoseSequenceCount()).toBe(2);

            const emptyContainer = mutatorWs.newBlock(
                "play_pose_sequence_container",
            );
            restored.compose(emptyContainer);
            expect(restored.getPoseSequenceCount()).toBe(1);
            mutatorWs.dispose();

            const customDropdown = new CustomFieldDropdown([["A", "a"]]);
            expect(customDropdown.getValue()).toBe("a");
        } finally {
            workspace.dispose();
            Blockly.Events.enable();
        }
    });
});
