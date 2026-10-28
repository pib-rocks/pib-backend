import { Block } from "blockly/core/block";
import { Order, pythonGenerator } from "blockly/python";
import {
  get_hand_x,
  get_hand_y,
  get_hand_z,
  motor_current,
  move_motor,
  move_motors,
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
    // Real Blockly substitutes its function-name placeholder before storing the
    // definition; the mock has to do the same or an assertion on "def <name>"
    // can never match.
    const resolved = code
      .split(generator.FUNCTION_NAME_PLACEHOLDER_)
      .join(name);
    definitions[`FN_${name}`] = resolved;
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

type MoveMotorsRow = { motor: string; angle: string };

function createMoveMotorsBlock(rows: MoveMotorsRow[]): Block {
  return {
    getMotorRowCount: () => rows.length,
    getFieldValue: (field: string) => {
      const match = /^MOTOR_(\d+)$/.exec(field);
      if (match) {
        return rows[Number(match[1])].motor;
      }
      throw new Error(`unexpected field ${field}`);
    },
  } as unknown as Block;
}

function createMoveMotorsGenerator(rows: MoveMotorsRow[]): MockGenerator {
  const generator = createMockGenerator();
  generator.valueToCode = (_block, name) => {
    const match = /^ANGLE_(\d+)$/.exec(name);
    if (match) {
      return rows[Number(match[1])].angle;
    }
    throw new Error(`unexpected input ${name}`);
  };
  return generator;
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
    const defs = Object.values(generator.definitions_).join("\n");
    expect(defs).toContain("get_joint_position_client.call_async");
    expect(defs).toContain("return centidegrees_to_degrees(response.position)");
  });

  it("maps TILT_FORWARD_HEAD to tilt_forward_motor", () => {
    const generator = createMockGenerator("0");
    const block = createMotorBlock("TILT_FORWARD_HEAD", "ABSOLUTE");
    const code = move_motor(block, generator);
    expect(code).toContain('"tilt_forward_motor"');
  });
});

describe("move_motors generator", () => {
  it("emits one vector move call for two motors in block order", () => {
    const rows = [
      { motor: "ELBOW_LEFT", angle: "12.5" },
      { motor: "WRIST_LEFT", angle: "-3" },
    ];
    const generator = createMoveMotorsGenerator(rows);
    const code = move_motors(createMoveMotorsBlock(rows), generator);
    const defs = Object.values(generator.definitions_).join("\n");

    expect(code).toBe(
      'move_motors("elbow_left", 12.5, "wrist_left", -3)\n',
    );
    expect(code.match(/move_motors\(/g)?.length ?? 0).toBe(1);
    expect(defs).toContain("write.move(*args)");
    expect(defs).not.toContain("apply_joint_trajectory");
    expect(defs).not.toContain("get_joint_position_client");
  });

  it("emits one vector move call for three motors", () => {
    const rows = [
      { motor: "TURN_HEAD", angle: "0" },
      { motor: "TILT_FORWARD_HEAD", angle: "5" },
      { motor: "ELBOW_RIGHT", angle: "90" },
    ];
    const generator = createMoveMotorsGenerator(rows);
    const code = move_motors(createMoveMotorsBlock(rows), generator);
    const defs = Object.values(generator.definitions_).join("\n");

    expect(code).toBe(
      'move_motors("turn_head_motor", 0, "tilt_forward_motor", 5, "elbow_right", 90)\n',
    );
    expect(code.match(/move_motors\(/g)?.length ?? 0).toBe(1);
    expect(defs).toContain("write.move(*args)");
    expect(defs).not.toContain("apply_joint_trajectory");
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
    expect(defs).toContain("from pib_sdk import ik_deg, Write, right_arm, left_arm");
    expect(defs).toContain("from pib_sdk.kinematics import from_rep103, to_rep103");
    expect(defs).toContain("target = list(from_rep103([x, y, z]))");
    expect(defs).toContain("q_deg = ik_deg(side, xyz=target)");
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
    expect(defs).toContain("from pib_sdk.kinematics import from_rep103, to_rep103");
    expect(defs).toContain("current = get_hand_position_xyz(side)");
    expect(defs).toContain("offset = list(from_rep103([x, y, z]))");
    expect(defs).toContain("current[0] + offset[0]");

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

      expect(code).toBe(
        `get_hand_position_rep103("left")[${index}]`,
      );
      expect(order).toBe(Order.MEMBER);
      const defs = Object.values(generator.definitions_).join("\n");
      expect(defs).toContain("from pib_sdk import get_hand_position_xyz");
      expect(defs).toContain("from pib_sdk.kinematics import from_rep103, to_rep103");
    },
  );

  it("uses the selected side", () => {
    const generator = createMockGenerator();
    const block = {
      getFieldValue: () => "right",
    } as unknown as Block;

    const [code] = get_hand_y(block, generator) as [string, Order];

    expect(code).toBe('get_hand_position_rep103("right")[1]');
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
