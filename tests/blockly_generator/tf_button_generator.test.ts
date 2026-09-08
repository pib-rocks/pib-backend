import { Block } from "blockly/core/block";
import { Order, pythonGenerator } from "blockly/python";
import {
  tf_button_set_color,
  tf_button_set_color_from_var,
} from "../../pib_blockly/pib_blockly_server/src/pib-blockly/program-generators/tf-button-generator";

type MockGenerator = typeof pythonGenerator & {
  definitions_: Record<string, string>;
  valueToCode: (
    block: Block,
    name: string,
    order: Order,
  ) => string;
};

function createMockGenerator(colorCode = "item"): MockGenerator {
  const definitions: Record<string, string> = {};
  const generator = Object.create(pythonGenerator) as MockGenerator;
  generator.definitions_ = definitions;
  generator.valueToCode = (_block, name, _order) => {
    if (name === "COLOR") return colorCode;
    return "";
  };
  return generator;
}

describe("tf_button_set_color_from_var generator", () => {
  it("emits set_button_color with RGB derived from a colour variable at runtime", () => {
    const generator = createMockGenerator("item");
    const block = {
      getFieldValue: (field: string) => {
        if (field === "BUTTON_ID") return "2";
        throw new Error(`unexpected field ${field}`);
      },
    } as unknown as Block;

    const code = tf_button_set_color_from_var(block, generator);

    expect(code).toBe(
      "blockly_client.set_button_color(2, int(item[1:3], 16), int(item[3:5], 16), int(item[5:7], 16))\n",
    );
    expect(Object.values(generator.definitions_).join("\n")).toContain(
      "blockly_client",
    );
  });
});

describe("tf_button_set_color generator", () => {
  it("still emits static RGB from the inline colour field", () => {
    const generator = createMockGenerator();
    const block = {
      getFieldValue: (field: string) => {
        if (field === "BUTTON_ID") return "1";
        if (field === "COLOR") return "#ff0000";
        throw new Error(`unexpected field ${field}`);
      },
    } as unknown as Block;

    const code = tf_button_set_color(block, generator);

    expect(code).toBe("blockly_client.set_button_color(1, 255, 0, 0)\n");
  });
});
