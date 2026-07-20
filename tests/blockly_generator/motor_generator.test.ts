import { Block } from "blockly/core/block";
import { Order, pythonGenerator } from "blockly/python";
import { move_motor } from "../../pib_blockly/pib_blockly_server/src/pib-blockly/program-generators/motor-generators";

type MockGenerator = typeof pythonGenerator & {
  definitions_: Record<string, string>;
  provideFunction_: (name: string, code: string) => string;
  valueToCode: (
    block: Block,
    name: string,
    order: Order,
  ) => string | [string, Order];
};

function createMockGenerator(positionCode = "1000"): MockGenerator {
  const definitions: Record<string, string> = {};
  const generator = Object.create(pythonGenerator) as MockGenerator;
  generator.definitions_ = definitions;
  generator.provideFunction_ = (name: string, code: string) => {
    definitions[`FN_${name}`] = code;
    return name;
  };
  generator.valueToCode = () => positionCode;
  return generator;
}

function createMotorBlock(
  motorOption: string,
  mode: "ABSOLUTE" | "RELATIVE",
): Block {
  return {
    getFieldValue: (field: string) => {
      if (field === "MOTORNAME") return motorOption;
      if (field === "MODE") return mode;
      throw new Error(`unexpected field ${field}`);
    },
  } as unknown as Block;
}

describe("move_motor generator", () => {
  it("generates absolute apply_joint_trajectory for elbow_left", () => {
    const generator = createMockGenerator("1000");
    const block = createMotorBlock("ELBOW_LEFT", "ABSOLUTE");
    const code = move_motor(block, generator);

    expect(code).toBe('apply_joint_trajectory("elbow_left", 1000)\n');
    expect(Object.values(generator.definitions_).join("\n")).toContain(
      "apply_joint_trajectory_client.call_async",
    );
    expect(Object.values(generator.definitions_).join("\n")).toContain("import rclpy");
  });

  it("generates relative position using get_joint_position", () => {
    const generator = createMockGenerator("250");
    const block = createMotorBlock("TURN_HEAD", "RELATIVE");
    const code = move_motor(block, generator);

    expect(code).toBe(
      'apply_joint_trajectory("turn_head_motor", get_joint_position(\'turn_head_motor\') + 250)\n',
    );
    expect(Object.values(generator.definitions_).join("\n")).toContain(
      "get_joint_position_client.call_async",
    );
  });

  it("maps TILT_FORWARD_HEAD to tilt_forward_motor", () => {
    const generator = createMockGenerator("0");
    const block = createMotorBlock("TILT_FORWARD_HEAD", "ABSOLUTE");
    const code = move_motor(block, generator);
    expect(code).toContain('"tilt_forward_motor"');
  });
});
