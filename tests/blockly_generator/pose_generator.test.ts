import { Block } from "blockly/core/block";
import { pythonGenerator } from "blockly/python";
import { moveToPoseGenerator } from "../../pib_blockly/pib_blockly_server/src/pib-blockly/program-generators/pose-generator";

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
    expect(defs).toContain("apply_joint_trajectory_client.call_async");
  });
});
