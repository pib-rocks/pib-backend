import * as Blockly from "blockly";
import { Block } from "blockly/core/block";
import { pythonGenerator } from "blockly/python";
import { setVolume } from "../../pib_blockly/pib_blockly_server/src/pib-blockly/program-blocks/set-volume-block";
import { set_volume } from "../../pib_blockly/pib_blockly_server/src/pib-blockly/program-generators/set-volume-generator";

type MockGenerator = typeof pythonGenerator & {
  definitions_: Record<string, string>;
  provideFunction_: (name: string, code: string) => string;
  valueToCode: (block: unknown, name: string, order: number) => string;
};

function createMockGenerator(percent: string): MockGenerator {
  const definitions: Record<string, string> = {};
  const generator = Object.create(pythonGenerator) as MockGenerator;
  generator.definitions_ = definitions;
  generator.provideFunction_ = (name: string, code: string) => {
    definitions[`FN_${name}`] = code;
    return name;
  };
  generator.valueToCode = (_block, name) => {
    expect(name).toBe("PERCENT");
    return percent;
  };
  return generator;
}

describe("set_volume generator", () => {
  it("initialises the service client and calls it with the input value", () => {
    const generator = createMockGenerator("volume_percent");
    const code = set_volume({} as Block, generator);
    const definitions = Object.values(generator.definitions_).join("\n");

    expect(code).toBe("set_volume(volume_percent)\n");
    expect(definitions).toContain("from datatypes.srv import SetVolume");
    expect(definitions).toContain(
      "set_volume_client = node.create_client(\n    SetVolume,\n    'set_volume'\n)",
    );
    expect(definitions).toContain("set_volume_client.wait_for_service()");
    expect(definitions).toContain("request.percent = int(percent)");
    expect(definitions).toContain(
      "future = set_volume_client.call_async(request)",
    );
    expect(definitions).toContain(
      "rclpy.spin_until_future_complete(node, future)",
    );
  });
});

describe("set_volume block", () => {
  beforeAll(() => Blockly.common.defineBlocks(setVolume));

  it("defines an Audio-coloured statement with a numeric percentage input", () => {
    const workspace = new Blockly.Workspace();
    Blockly.Events.disable();
    try {
      const block = workspace.newBlock("set_volume");
      expect(block.getInput("PERCENT")).toBeTruthy();
      expect(block.getInput("PERCENT")?.connection?.getCheck()).toEqual([
        "Number",
      ]);
      expect(block.getColour()).toBe("#745ba5");
      expect(block.previousConnection).toBeTruthy();
      expect(block.nextConnection).toBeTruthy();
    } finally {
      workspace.dispose();
      Blockly.Events.enable();
    }
  });
});
