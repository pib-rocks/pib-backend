import { Block } from "blockly/core/block";
import { Order, pythonGenerator } from "blockly/python";
import {
  start_model,
  stop_model,
} from "../../pib_blockly/pib_blockly_server/src/pib-blockly/program-generators/model-generators";
import {
  getModelDropdownOptions,
  getStopModelDropdownOptions,
  STOP_ALL_MODELS_VALUE,
} from "../../pib_blockly/pib_blockly_server/src/pib-blockly/program-blocks/model-blocks";

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

  it("stops every model this program owns when All is selected", () => {
    const generator = createMockGenerator();
    const code = stop_model(modelFieldBlock(STOP_ALL_MODELS_VALUE), generator);

    expect(code).toBe("stop_all_models_with_sdk()\n");
    expect(code).not.toMatch(/stop_model\(/);
    expect(code).not.toMatch(/'[^']+'/);
    expect(code).not.toMatch(/"[^"]+"/);

    const defs = joinedDefs(generator);
    expect(defs).toContain("from pib_sdk import Models");
    expect(defs).toContain("models.stop_all_models()");
    expect(defs).not.toMatch(/stop_all_models\([^)]+\)/);
    expect(defs).not.toContain("models.stop_model(");
  });
});

describe("stop_model dropdown", () => {
  it("always offers All with a sentinel that cannot collide with a model id", () => {
    const stopOptions = getStopModelDropdownOptions();
    const startOptions = getModelDropdownOptions();

    expect(stopOptions[0]).toEqual(["All", STOP_ALL_MODELS_VALUE]);
    expect(STOP_ALL_MODELS_VALUE.startsWith("__")).toBe(true);
    expect(stopOptions.filter(([label]) => label === "All")).toHaveLength(1);
    expect(startOptions.some(([, value]) => value === STOP_ALL_MODELS_VALUE)).toBe(
      false,
    );
  });
});
