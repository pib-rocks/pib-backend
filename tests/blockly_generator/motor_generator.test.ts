import { Block } from "blockly/core/block";
import { Order, pythonGenerator } from "blockly/python";
import {
  get_hand_x,
  get_hand_y,
  get_hand_z,
  motor_current,
  move_motor,
  set_hand_position_xyz,
} from "../../pib_blockly/pib_blockly_server/src/pib-blockly/program-generators/motor-generators";

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

describe("motor_current generator", () => {
  it("maps SHOULDER_VERTICAL_LEFT and emits get_current_ma", () => {
    const generator = createMockGenerator();
    const block = {
      getFieldValue: (field: string) => {
        if (field === "MOTORNAME") return "SHOULDER_VERTICAL_LEFT";
        throw new Error(`unexpected field ${field}`);
      },
    } as unknown as Block;

    const [code, order] = motor_current(block, generator);

    expect(code).toBe('get_motor_current_ma("shoulder_vertical_left")');
    expect(order).toBe(Order.FUNCTION_CALL);
    const defs = Object.values(generator.definitions_).join("\n");
    expect(defs).toContain("from pib_sdk.telemetry import Telemetry");
    expect(defs).toContain('os.getenv("ROSBRIDGE_HOST", "rosbridge-ws")');
    expect(defs).toContain("get_current_ma(");
    expect(defs).toContain('Telemetry(host="localhost", port=9090)');
  });

  it("rejects an unknown motor option", () => {
    const generator = createMockGenerator();
    const block = {
      getFieldValue: () => "NOT_A_MOTOR",
    } as unknown as Block;

    expect(() => motor_current(block, generator)).toThrow(
      "'NOT_A_MOTOR' is not a valid value for 'MOTORNAME'.",
    );
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
    expect(defs).toContain("from pib_sdk import get_hand_position_xyz");
    expect(defs).toContain("current = get_hand_position_xyz(side)");
    expect(defs).toContain(
      "target = [current[0] + x, current[1] + y, current[2] + z]",
    );

    // the inline forward-kinematics helper was replaced by the sdk call
    expect(defs).not.toContain("get_current_hand_position");
    expect(defs).not.toContain("from pib_sdk import fk, get_arm_model");
    expect(defs).not.toContain("get_arm_model(side).motor_names");
    expect(defs).not.toContain("fk(side, q_deg).translation");
    expect(defs).not.toContain("get_joint_position_client.call_async");
  });
});

describe("get_hand_x/y/z generators", () => {
  const cases: [string, (block: Block, generator: MockGenerator) => unknown, number][] = [
    ["get_hand_x", get_hand_x, 0],
    ["get_hand_y", get_hand_y, 1],
    ["get_hand_z", get_hand_z, 2],
  ];

  it.each(cases)(
    "%s reads the sdk hand position and indexes component %#",
    (_name, blockGenerator, index) => {
      const generator = createMockGenerator();
      const block = {
        getFieldValue: (field: string) => {
          if (field === "SIDE") return "left";
          throw new Error(`unexpected field ${field}`);
        },
      } as unknown as Block;

      const [code, order] = blockGenerator(block, generator) as [string, Order];

      expect(code).toBe(`get_hand_position_xyz("left")[${index}]`);
      expect(order).toBe(Order.MEMBER);
      expect(Object.values(generator.definitions_).join("\n")).toContain(
        "from pib_sdk import get_hand_position_xyz",
      );
    },
  );

  it("uses the selected side", () => {
    const generator = createMockGenerator();
    const block = {
      getFieldValue: () => "right",
    } as unknown as Block;

    const [code] = get_hand_y(block, generator) as [string, Order];

    expect(code).toBe('get_hand_position_xyz("right")[1]');
  });

  it("rejects an invalid side", () => {
    const generator = createMockGenerator();
    const block = {
      getFieldValue: () => "middle",
    } as unknown as Block;

    expect(() => get_hand_x(block, generator)).toThrow(
      "'middle' is not a valid value for 'SIDE'.",
    );
  });
});
