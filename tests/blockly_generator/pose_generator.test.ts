import * as Blockly from "blockly";
import { Block } from "blockly/core/block";
import { Order, pythonGenerator } from "blockly/python";
import {
  get_all_poses,
  get_pose_joints,
  has_pose,
  moveToPoseGenerator,
  pose_count,
  save_current_pose,
} from "../../pib_blockly/pib_blockly_server/src/pib-blockly/program-generators/pose-generator";
import { poseBlocks } from "../../pib_blockly/pib_blockly_server/src/pib-blockly/program-blocks/pose-block";

type MockGenerator = typeof pythonGenerator & {
  definitions_: Record<string, string>;
  provideFunction_: (name: string, code: string) => string;
};

function createMockGenerator(): MockGenerator {
  const definitions: Record<string, string> = {};
  const generator = Object.create(pythonGenerator) as MockGenerator;
  generator.definitions_ = definitions;
  generator.provideFunction_ = (name: string, code: string) => {
    definitions[`FN_${name}`] = code;
    return name;
  };
  return generator;
}

describe("moveToPoseGenerator", () => {
  it("generates apply_pose call with pose UUID", () => {
    const poseId = "aaaaaaaa-bbbb-cccc-dddd-eeeeeeeeeeee";
    const block = {
      getFieldValue: (field: string) => {
        if (field === "POSE") return poseId;
        throw new Error(`unexpected field ${field}`);
      },
    } as unknown as Block;

    const generator = createMockGenerator();
    const code = moveToPoseGenerator(block, generator);

    expect(code).toBe(`apply_pose("${poseId}")\n`);
    const defs = Object.values(generator.definitions_).join("\n");
    expect(defs).toContain("from pib_api_client import pose_client");
    expect(defs).toContain("pib_sdk.Write");
    expect(defs).toContain("import pib_sdk");
  });
});

function blockWithName(name: string): Block {
  return {
    getFieldValue: (field: string) => {
      if (field === "NAME") return name;
      throw new Error(`unexpected field ${field}`);
    },
  } as unknown as Block;
}

describe("pose SDK generators", () => {
  it("saves every motor through All and explicitly includes the head", () => {
    const generator = createMockGenerator();
    const code = save_current_pose(blockWithName('rest "pose"'), generator);

    expect(code).toBe(
      "save_current_pose_with_all_motors('rest \"pose\"')\n",
    );
    const defs = Object.values(generator.definitions_).join("\n");
    expect(defs).toContain(
      "from pib_sdk.features.poses import save_current_pose, list_poses, get_pose",
    );
    expect(defs).toContain(
      "from pib_sdk.control import All, _expand_motor_specs",
    );
    expect(defs).toContain("motor_names = _expand_motor_specs([All])");
    expect(defs).toContain('"turn_head_motor" in motor_names');
    expect(defs).toContain('"tilt_forward_motor" in motor_names');
    expect(defs).toContain(
      "save_current_pose(telemetry, pose_backend, name, motor_names)",
    );
  });

  it("generates the all-poses list expression", () => {
    const generator = createMockGenerator();
    expect(get_all_poses({} as Block, generator)).toEqual([
      "list_poses(pose_backend)",
      Order.FUNCTION_CALL,
    ]);
  });

  it("generates the named pose joints map expression", () => {
    const generator = createMockGenerator();
    expect(get_pose_joints(blockWithName("wave"), generator)).toEqual([
      "get_pose(pose_backend, name='wave').motor_angles_deg",
      Order.MEMBER,
    ]);
  });

  it("generates a boolean pose existence expression", () => {
    const generator = createMockGenerator();
    expect(has_pose(blockWithName("wave"), generator)).toEqual([
      "any(pose.name == 'wave' for pose in list_poses(pose_backend))",
      Order.FUNCTION_CALL,
    ]);
  });

  it("generates the pose count expression", () => {
    const generator = createMockGenerator();
    expect(pose_count({} as Block, generator)).toEqual([
      "len(list_poses(pose_backend))",
      Order.FUNCTION_CALL,
    ]);
  });

  it("initializes the SDK backend from the configured Flask API URL", () => {
    const generator = createMockGenerator();
    get_all_poses({} as Block, generator);
    const defs = Object.values(generator.definitions_).join("\n");

    expect(defs).toContain("from pib_sdk.backend import BackendClient");
    expect(defs).toContain(
      'os.getenv("FLASK_API_BASE_URL", "http://flask-app:5000")',
    );
    expect(defs).toContain("pose_backend = BackendClient(");
  });
});

describe("save current pose block", () => {
  beforeAll(() => Blockly.common.defineBlocks(poseBlocks));

  it("has one name field and no arm or range dropdown", () => {
    const workspace = new Blockly.Workspace();
    Blockly.Events.disable();
    try {
      const block = workspace.newBlock("save_current_pose");

      expect(block.getField("NAME")).toBeInstanceOf(Blockly.FieldTextInput);
      expect(block.getField("SIDE")).toBeNull();
      expect(block.getField("RANGE")).toBeNull();
      expect(
        block.inputList.flatMap((input) => input.fieldRow).some(
          (field) => field instanceof Blockly.FieldDropdown,
        ),
      ).toBe(false);
    } finally {
      workspace.dispose();
      Blockly.Events.enable();
    }
  });
});
