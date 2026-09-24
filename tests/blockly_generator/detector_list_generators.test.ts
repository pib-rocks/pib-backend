import * as Blockly from "blockly";
import { Block } from "blockly/core/block";
import { Order, pythonGenerator } from "blockly/python";
import { modelBlocks } from "../../pib_blockly/pib_blockly_server/src/pib-blockly/program-blocks/model-blocks";
import {
  get_emotion_detections,
  get_head_pose_detections,
  get_object_detections,
  get_qr_detections,
} from "../../pib_blockly/pib_blockly_server/src/pib-blockly/program-generators/model-generators";

type MockGenerator = typeof pythonGenerator & {
  definitions_: Record<string, string>;
};

function createMockGenerator(): MockGenerator {
  const generator = Object.create(pythonGenerator) as MockGenerator;
  generator.definitions_ = {};
  generator.provideFunction_ = (name: string, code: string) => {
    generator.definitions_[`FN_${name}`] = code.replace(
      generator.FUNCTION_NAME_PLACEHOLDER_,
      name,
    );
    return name;
  };
  return generator;
}

const SIX_FIELD_LIST = `[
            detection.label,
            detection.score,
            detection.x_min,
            detection.y_min,
            detection.x_max,
            detection.y_max,
        ]`;

const NINE_FIELD_LIST = `[
            detection.label,
            detection.score,
            detection.x_min,
            detection.y_min,
            detection.x_max,
            detection.y_max,
            _scalar(detection, "yaw_deg", "yaw"),
            _scalar(detection, "pitch_deg", "pitch"),
            _scalar(detection, "roll_deg", "roll"),
        ]`;

function expectReporterBlock(
  type: string,
  tooltipFragments: string[],
): void {
  const previousDefinition = Blockly.Blocks[type];
  Blockly.common.defineBlocks({ [type]: modelBlocks[type] });
  Blockly.Events.disable();
  const workspace = new Blockly.Workspace();

  try {
    const block = workspace.newBlock(type);
    expect(block.outputConnection?.getCheck()).toEqual(["Array"]);
    for (const fragment of tooltipFragments) {
      expect(block.getTooltip()).toContain(fragment);
    }
  } finally {
    workspace.dispose();
    Blockly.Events.enable();
    if (previousDefinition) {
      Blockly.Blocks[type] = previousDefinition;
    } else {
      delete Blockly.Blocks[type];
    }
  }
}

function generatedPython(
  generate: (block: Block, generator: MockGenerator) => [string, Order],
): string {
  const generator = createMockGenerator();
  generate({} as Block, generator);
  return Object.values(generator.definitions_).join("\n");
}

describe("get_object_detections", () => {
  it("defines a list reporter whose tooltip documents all six indices", () => {
    expectReporterBlock("get_object_detections", [
      "[label, score, x_min, y_min, x_max, y_max]",
      "indices 0 through 5",
    ]);
  });

  it("generates a fresh topic subscription and a flat six-field list per object", () => {
    const generator = createMockGenerator();
    const [code, order] = get_object_detections({} as Block, generator);

    expect(code).toBe("get_object_detections()");
    expect(order).toBe(Order.FUNCTION_CALL);

    const python = Object.values(generator.definitions_).join("\n");
    expect(python).toContain('"/detections/yolov6n_coco_640x640"');
    expect(python).toContain("subscription = node.create_subscription(");
    expect(python).toContain("node.destroy_subscription(subscription)");
    expect(python).toContain(SIX_FIELD_LIST);
    expect(python).toContain("for detection in message.detections");
  });

  it("generates an empty-list fallback and tears down the subscription", () => {
    const python = generatedPython(get_object_detections);
    expect(python).toContain(`if message is None:
        logging.warning("no object detections received")
        return []`);
    expect(python).toContain("for detection in message.detections");
    expect(python).toContain("node.destroy_subscription(subscription)");
  });
});

describe("get_qr_detections", () => {
  it("defines a list reporter whose tooltip documents all six indices", () => {
    expectReporterBlock("get_qr_detections", [
      "[label, score, x_min, y_min, x_max, y_max]",
      "indices 0 through 5",
    ]);
  });

  it("generates a fresh topic subscription and a flat six-field list per code", () => {
    const generator = createMockGenerator();
    const [code, order] = get_qr_detections({} as Block, generator);

    expect(code).toBe("get_qr_detections()");
    expect(order).toBe(Order.FUNCTION_CALL);

    const python = Object.values(generator.definitions_).join("\n");
    expect(python).toContain('"/detections/qr_code_detection_384x384"');
    expect(python).toContain("subscription = node.create_subscription(");
    expect(python).toContain("node.destroy_subscription(subscription)");
    expect(python).toContain(SIX_FIELD_LIST);
    expect(python).toContain("for detection in message.detections");
  });

  it("generates an empty-list fallback and tears down the subscription", () => {
    const python = generatedPython(get_qr_detections);
    expect(python).toContain(`if message is None:
        logging.warning("no qr detections received")
        return []`);
    expect(python).toContain("for detection in message.detections");
    expect(python).toContain("node.destroy_subscription(subscription)");
  });
});

describe("get_emotion_detections", () => {
  it("defines a list reporter whose tooltip documents all six indices", () => {
    expectReporterBlock("get_emotion_detections", [
      "[label, score, x_min, y_min, x_max, y_max]",
      "indices 0 through 5",
    ]);
  });

  it("generates a fresh topic subscription and a flat six-field list per face", () => {
    const generator = createMockGenerator();
    const [code, order] = get_emotion_detections({} as Block, generator);

    expect(code).toBe("get_emotion_detections()");
    expect(order).toBe(Order.FUNCTION_CALL);

    const python = Object.values(generator.definitions_).join("\n");
    expect(python).toContain('"/detections/emotion_recognition_crop"');
    expect(python).toContain("subscription = node.create_subscription(");
    expect(python).toContain("node.destroy_subscription(subscription)");
    expect(python).toContain(SIX_FIELD_LIST);
    expect(python).toContain("for detection in message.detections");
  });

  it("generates an empty-list fallback and tears down the subscription", () => {
    const python = generatedPython(get_emotion_detections);
    expect(python).toContain(`if message is None:
        logging.warning("no emotion detections received")
        return []`);
    expect(python).toContain("for detection in message.detections");
    expect(python).toContain("node.destroy_subscription(subscription)");
  });
});

describe("get_head_pose_detections", () => {
  it("defines a list reporter whose tooltip documents all nine indices", () => {
    expectReporterBlock("get_head_pose_detections", [
      "[label, score, x_min, y_min, x_max, y_max, yaw_deg, pitch_deg, roll_deg]",
      "indices 0 through 8",
    ]);
  });

  it("generates a fresh topic subscription and a flat nine-field list per face", () => {
    const generator = createMockGenerator();
    const [code, order] = get_head_pose_detections({} as Block, generator);

    expect(code).toBe("get_head_pose_detections()");
    expect(order).toBe(Order.FUNCTION_CALL);

    const python = Object.values(generator.definitions_).join("\n");
    expect(python).toContain('"/detections/head_pose_estimation_crop"');
    expect(python).toContain("subscription = node.create_subscription(");
    expect(python).toContain("node.destroy_subscription(subscription)");
    expect(python).toContain(NINE_FIELD_LIST);
    expect(python).toContain("for detection in message.detections");
    expect(python).toContain("detection.scalar_values");
  });

  it("generates an empty-list fallback and tears down the subscription", () => {
    const python = generatedPython(get_head_pose_detections);
    expect(python).toContain(`if message is None:
        logging.warning("no head pose detections received")
        return []`);
    expect(python).toContain("for detection in message.detections");
    expect(python).toContain("node.destroy_subscription(subscription)");
  });
});
