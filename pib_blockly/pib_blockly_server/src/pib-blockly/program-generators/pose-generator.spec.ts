import * as Blockly from "blockly";
import {Block} from "blockly/core/block";
import {Order, pythonGenerator} from "blockly/python";
import {
    moveToPose,
    PlayPoseSequenceBlock,
    poseBlocks,
} from "../program-blocks/pose-block";
import {
    get_all_poses,
    get_pose_joints,
    has_pose,
    moveToPoseGenerator,
    play_pose_sequence,
    pose_count,
    save_current_pose,
    save_detection_as_pose,
} from "./pose-generator";

type MockGenerator = typeof pythonGenerator & {
    definitions_: Record<string, string>;
};

function createGenerator(values: Record<string, string> = {}): MockGenerator {
    const generator = Object.create(pythonGenerator) as MockGenerator;
    generator.definitions_ = {};
    generator.provideFunction_ = (name: string, code: string | string[]) => {
        const declaration = Array.isArray(code) ? code.join("\n") : code;
        generator.definitions_[`function_${name}`] = declaration.replace(
            generator.FUNCTION_NAME_PLACEHOLDER_,
            name,
        );
        return name;
    };
    generator.valueToCode = (_block: Block, name: string, _order: number) =>
        values[name] ?? "";
    return generator;
}

describe("pose generators", () => {
    beforeAll(() => {
        Blockly.common.defineBlocks(moveToPose);
        Blockly.common.defineBlocks(poseBlocks);
    });

    it("generates Python code for dynamic play_pose_sequence blocks and legacy inputs", () => {
        Blockly.Blocks["move_to_pose"].getPoses = () => [
            ["Pose 1", "pose-1"],
            ["Pose 2", "pose-2"],
        ];
        const workspace = new Blockly.Workspace();
        Blockly.Events.disable();
        try {
            const seqBlock = workspace.newBlock(
                "play_pose_sequence",
            ) as PlayPoseSequenceBlock;
            seqBlock.setFieldValue("pose-1", "POSE_0");
            seqBlock.setFieldValue(1.5, "DURATION_0");
            seqBlock.setFieldValue("pose-2", "POSE_1");
            seqBlock.setFieldValue(2, "DURATION_1");

            const generator = createGenerator();
            const code = play_pose_sequence(seqBlock, generator);
            expect(code).toBe(
                '_pib_play_pose_sequence_timed([("pose-1", 1.5), ("pose-2", 2)])\n',
            );

            const mockCountBlock = {
                itemCount_: 1,
                getFieldValue: (name: string) =>
                    name === "POSE_0" ? "pose-1" : 0,
            } as unknown as Block;
            expect(play_pose_sequence(mockCountBlock, generator)).toBe(
                '_pib_play_pose_sequence_timed([("pose-1", 1.5)])\n',
            );

            const mockInputBlock = {
                getInput: (name: string) => (name === "STEP_0" ? {} : null),
                getFieldValue: (name: string) =>
                    name === "POSE_0" ? "LOADING" : 1,
            } as unknown as Block;
            expect(play_pose_sequence(mockInputBlock, generator)).toBe(
                "_pib_play_pose_sequence_timed([])\n",
            );

            const legacyGen = createGenerator({
                SEQUENCE: "[['w1', 1.0]]",
            });
            expect(play_pose_sequence({} as Block, legacyGen)).toBe(
                "_pib_play_pose_sequence_timed([['w1', 1.0]])\n",
            );
        } finally {
            workspace.dispose();
            Blockly.Events.enable();
        }
    });

    it("generates move_to_pose and pose query expressions", () => {
        const generator = createGenerator({NAME: "'my_pose'"});
        const moveBlock = {
            getFieldValue: () => "pose-uuid-1",
        } as unknown as Block;
        expect(moveToPoseGenerator(moveBlock, generator)).toBe(
            'apply_pose("pose-uuid-1")\n',
        );
        expect(save_current_pose({} as Block, generator)).toBe(
            "save_current_pose_with_all_motors('my_pose')\n",
        );
        expect(save_detection_as_pose({} as Block, createGenerator())).toBe(
            'save_current_pose_with_all_motors("pose name")\n',
        );
        expect(get_all_poses({} as Block, generator)).toEqual([
            "list_poses(pose_backend)",
            Order.FUNCTION_CALL,
        ]);
        const nameBlock = {
            getFieldValue: () => "wave'1",
        } as unknown as Block;
        expect(get_pose_joints(nameBlock, generator)).toEqual([
            String.raw`get_pose(pose_backend, name='wave\'1').motor_angles_deg`,
            Order.MEMBER,
        ]);
        expect(has_pose(nameBlock, generator)).toEqual([
            String.raw`any(pose.name == 'wave\'1' for pose in list_poses(pose_backend))`,
            Order.FUNCTION_CALL,
        ]);
        expect(pose_count({} as Block, generator)).toEqual([
            "len(list_poses(pose_backend))",
            Order.FUNCTION_CALL,
        ]);
    });
});
