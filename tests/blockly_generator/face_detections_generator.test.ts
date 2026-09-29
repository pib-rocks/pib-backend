import * as Blockly from "blockly";
import { Block } from "blockly/core/block";
import { Order, pythonGenerator } from "blockly/python";
import { modelBlocks } from "../../pib_blockly/pib_blockly_server/src/pib-blockly/program-blocks/model-blocks";
import { get_face_detections } from "../../pib_blockly/pib_blockly_server/src/pib-blockly/program-generators/model-generators";

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

describe("get_face_detections", () => {
  it("defines a list reporter whose tooltip documents all six indices", () => {
    const previousDefinition = Blockly.Blocks.get_face_detections;
    Blockly.common.defineBlocks({
      get_face_detections: modelBlocks.get_face_detections,
    });
    Blockly.Events.disable();
    const workspace = new Blockly.Workspace();

    try {
      const block = workspace.newBlock("get_face_detections");
      expect(block.outputConnection?.getCheck()).toEqual(["Array"]);
      expect(block.getTooltip()).toContain(
        "[label, score, x_min, y_min, x_max, y_max]",
      );
      expect(block.getTooltip()).toContain("indices 0 through 5");
    } finally {
      workspace.dispose();
      Blockly.Events.enable();
      if (previousDefinition) {
        Blockly.Blocks.get_face_detections = previousDefinition;
      } else {
        delete Blockly.Blocks.get_face_detections;
      }
    }
  });

  it("generates a fresh topic subscription and a flat six-field list per face", () => {
    const generator = createMockGenerator();

    const [code, order] = get_face_detections({} as Block, generator);

    expect(code).toBe("get_face_detections()");
    expect(order).toBe(Order.FUNCTION_CALL);

    const python = Object.values(generator.definitions_).join("\n");
    expect(python).toContain("from datatypes.msg import DetectionArray");
    expect(python).toContain(
      '"/detections/face_detection_yunet_160x120"',
    );
    expect(python).toContain("subscription = node.create_subscription(");
    expect(python).toContain("node.destroy_subscription(subscription)");
    expect(python).toContain(`[
            detection.label,
            detection.score,
            detection.x_min,
            detection.y_min,
            detection.x_max,
            detection.y_max,
        ]`);
    expect(python).toContain("for detection in message.detections");
  });

  it("generates an empty-list fallback when no message or no faces exist", () => {
    const generator = createMockGenerator();
    get_face_detections({} as Block, generator);

    const python = Object.values(generator.definitions_).join("\n");
    expect(python).toContain(`if message is None:
        logging.warning("no face detections received")
        return []`);
    expect(python).toContain("for detection in message.detections");
  });
});
