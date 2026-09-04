import { Block } from "blockly/core/block";
import { Order, pythonGenerator } from "blockly/python";
import { move_motor, set_hand_position_xyz } from "../../pib_blockly/pib_blockly_server/src/pib-blockly/program-generators/motor-generators";

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

function createHandPositionBlock(
  side: "left" | "right",
  mode: "ABSOLUTE" | "RELATIVE",
): Block {
  return {
    getFieldValue: (field: string) => {
      if (field === "SIDE") return side;
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
      "pib_sdk.Write",
    );
    expect(Object.values(generator.definitions_).join("\n")).toContain("import pib_sdk");
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

describe("set_hand_position_xyz generator", () => {
  function createXyzGenerator() {
    const generator = createMockGenerator();
    generator.valueToCode = (_block, name) => {
      if (name === "X") return "0.2";
      if (name === "Y") return "0.1";
      if (name === "Z") return "0.3";
      throw new Error(`unexpected input ${name}`);
    };
    return generator;
  }

  it("generates an absolute XYZ target without reading current joints", () => {
    const generator = createXyzGenerator();
    const block = createHandPositionBlock("left", "ABSOLUTE");

    const code = set_hand_position_xyz(block, generator);

    expect(code).toBe(
      'set_hand_position_xyz("left", "ABSOLUTE", 0.2, 0.1, 0.3)\n',
    );
    const defs = Object.values(generator.definitions_).join("\n");
    expect(defs).toContain("from pib_sdk import ik, Write, right_arm, left_arm");
    expect(defs).toContain("target = [x, y, z]");
    expect(defs).toContain("q_deg = ik(side, xyz=target)");
    expect(defs).toContain("pib.move(arm, *q_deg)");
    expect(defs).toContain('os.getenv("ROSBRIDGE_HOST", "rosbridge-ws")');
    expect(defs).not.toContain("get_joint_position_client.call_async");
  });

  it("generates a relative XYZ target from the current hand position", () => {
    const generator = createXyzGenerator();
    const block = createHandPositionBlock("right", "RELATIVE");

    const code = set_hand_position_xyz(block, generator);

    expect(code).toBe(
      'set_hand_position_xyz("right", "RELATIVE", 0.2, 0.1, 0.3)\n',
    );
    const defs = Object.values(generator.definitions_).join("\n");
    expect(defs).toContain("get_joint_position_client.call_async");
    expect(defs).toContain("from pib_sdk import fk, get_arm_model");
    expect(defs).toContain("motor_names = get_arm_model(side).motor_names");
    expect(defs).toContain("get_joint_position(motor_name) / 100.0");
    expect(defs).toContain("return fk(side, q_deg).translation");
    expect(defs).toContain(
      "target = [current[0] + x, current[1] + y, current[2] + z]",
    );
  });
});
