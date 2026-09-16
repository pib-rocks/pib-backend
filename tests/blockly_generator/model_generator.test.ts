import { Block } from "blockly/core/block";
import { Order, pythonGenerator } from "blockly/python";
import {
  get_detection_field,
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

function emptyBlock(): Block {
  return {} as Block;
}

function fieldBlock(fieldValue: string): Block {
  return {
    getFieldValue: (field: string) => {
      if (field === "FIELD") return fieldValue;
      throw new Error(`unexpected field ${field}`);
    },
  } as unknown as Block;
}

function joinedDefs(generator: MockGenerator): string {
  return Object.values(generator.definitions_).join("\n");
}

describe("start_model generator", () => {
  it("starts the default hand_tracking model with registry shaves", () => {
    const generator = createMockGenerator();
    const code = start_model(emptyBlock(), generator);

    expect(code).toBe(
      '_blockly_model_manager.start("hand_tracking", 0)\n',
    );
    const defs = joinedDefs(generator);
    expect(defs).toContain(
      "from datatypes.srv import StartModel, StopModel",
    );
    expect(defs).toContain('node.create_client(StartModel, "/start_model")');
    expect(defs).toContain("atexit.register(self.release_all)");
    expect(defs).toContain("request.owner = self.owner");
    expect(defs).toContain(
      `_blockly_model_manager = BlocklyModelManager(node, f"blockly-{os.getpid()}")`,
    );
  });

  it("passes a connected model id and shave count through to start", () => {
    const generator = createMockGenerator({
      MODEL_ID: "selected_model",
      SHAVES: "6",
    });
    expect(start_model(emptyBlock(), generator)).toBe(
      "_blockly_model_manager.start(selected_model, 6)\n",
    );
  });
});

describe("stop_model generator", () => {
  it("stops the default hand_tracking model", () => {
    const generator = createMockGenerator();
    const code = stop_model(emptyBlock(), generator);

    expect(code).toBe('_blockly_model_manager.stop("hand_tracking")\n');
    const defs = joinedDefs(generator);
    expect(defs).toContain('node.create_client(StopModel, "/stop_model")');
    expect(defs).toContain("self.owned_models.discard(model_id)");
    expect(defs).toContain("for model_id in tuple(self.owned_models):");
  });

  it("stops a connected model id variable instead of the default", () => {
    const generator = createMockGenerator({ MODEL_ID: "model_to_release" });
    expect(stop_model(emptyBlock(), generator)).toBe(
      "_blockly_model_manager.stop(model_to_release)\n",
    );
  });
});

describe("get_detection_field generator", () => {
  it("reads the default label field from the first hand_tracking detection", () => {
    const generator = createMockGenerator();
    const [code, order] = get_detection_field(
      fieldBlock("label"),
      generator,
    );

    expect(code).toBe(
      "get_detection_field(\"hand_tracking\", 0, 'label', \"\")",
    );
    expect(order).toBe(Order.FUNCTION_CALL);
    const defs = joinedDefs(generator);
    expect(defs).toContain("from datatypes.msg import DetectionArray");
    expect(defs).toContain('f"/detections/{model_id}"');
    expect(defs).toContain(
      'if field in ("label", "score", "x_min", "y_min", "x_max", "y_max"):',
    );
  });

  it("selects a named keypoint axis from a connected detection index", () => {
    const generator = createMockGenerator({
      MODEL_ID: '"hand_tracking"',
      INDEX: "2",
      NAME: '"wrist"',
    });
    const [code] = get_detection_field(fieldBlock("keypoint_z"), generator);

    expect(code).toBe(
      "get_detection_field(\"hand_tracking\", 2, 'keypoint_z', \"wrist\")",
    );
    const defs = joinedDefs(generator);
    expect(defs).toContain(
      'if field in ("keypoint_x", "keypoint_y", "keypoint_z"):',
    );
    expect(defs).toContain("detection.keypoint_names.index(name)");
  });

  it("looks up a named scalar when the field dropdown is scalar_values", () => {
    const generator = createMockGenerator({
      MODEL_ID: "active_model",
      INDEX: "item_index",
      NAME: "scalar_name",
    });
    const [code] = get_detection_field(
      fieldBlock("scalar_values"),
      generator,
    );

    expect(code).toBe(
      "get_detection_field(active_model, item_index, 'scalar_values', scalar_name)",
    );
    expect(joinedDefs(generator)).toContain("detection.scalar_names.index(name)");
  });
});
