import { Block } from "blockly/core/block";
import { Order, pythonGenerator } from "blockly/python";
import {
  start_model,
  stop_model,
} from "../../pib_blockly/pib_blockly_server/src/pib-blockly/program-generators/model-generators";

type MockGenerator = typeof pythonGenerator & {
  definitions_: Record<string, string>;
  provideFunction_: (name: string, code: string) => string;
  valueToCode: (
    block: Block,
    name: string,
    order: Order,
  ) => string;
};

function createMockGenerator(
  values: Record<string, string> = {},
): MockGenerator {
  const definitions: Record<string, string> = {};
  const generator = Object.create(pythonGenerator) as MockGenerator;
  generator.definitions_ = definitions;
  generator.provideFunction_ = (name: string, code: string) => {
    definitions[`FN_${name}`] = code;
    return name;
  };
  generator.valueToCode = (_block, name, order) => {
    expect(order).toBe(Order.NONE);
    return values[name] ?? "";
  };
  return generator;
}

function modelFieldBlock(modelId: string): Block {
  return {
    getFieldValue: (field: string) => {
      if (field === "MODEL_ID") return modelId;
      throw new Error(`unexpected field ${field}`);
    },
  } as unknown as Block;
}

function joinedDefs(generator: MockGenerator): string {
  return Object.values(generator.definitions_).join("\n");
}

describe("start_model generator", () => {
  it("starts the selected model through the SDK", () => {
    const generator = createMockGenerator();
    const code = start_model(modelFieldBlock("hand_tracking"), generator);

    expect(code).toBe("start_model_with_sdk('hand_tracking')\n");
    const defs = joinedDefs(generator);
    expect(defs).toContain("from pib_sdk import Models");
    expect(defs).toContain(
      "with Models(host=rosbridge_host, port=9090) as models:",
    );
    expect(defs).toContain("models.start_model(str(model_id))");
    expect(defs).not.toContain("StartModel");
    expect(defs).not.toContain("rclpy");
  });
});

describe("stop_model generator", () => {
  it("stops the selected model through the SDK", () => {
    const generator = createMockGenerator();
    const code = stop_model(modelFieldBlock("face_detection"), generator);

    expect(code).toBe("stop_model_with_sdk('face_detection')\n");
    const defs = joinedDefs(generator);
    expect(defs).toContain("from pib_sdk import Models");
    expect(defs).toContain("models.stop_model(str(model_id))");
    expect(defs).not.toContain("StopModel");
    expect(defs).not.toContain("rclpy");
  });
});
