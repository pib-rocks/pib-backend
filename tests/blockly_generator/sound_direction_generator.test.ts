import { Block } from "blockly/core/block";
import { Order, pythonGenerator } from "blockly/python";
import { get_sound_direction } from "../../pib_blockly/pib_blockly_server/src/pib-blockly/program-generators/sound-direction-generator";

type MockGenerator = typeof pythonGenerator & {
  definitions_: Record<string, string>;
  provideFunction_: (name: string, code: string) => string;
};

function createMockGenerator(): MockGenerator {
  const definitions: Record<string, string> = {};
  const generator = Object.create(pythonGenerator) as MockGenerator;
  generator.definitions_ = definitions;
  generator.provideFunction_ = (name: string, code: string) => {
    definitions[`FN_${name}`] = code;
    return name;
  };
  return generator;
}

describe("get_sound_direction generator", () => {
  it("emits a helper that subscribes to /doa_angle", () => {
    const generator = createMockGenerator();
    const block = {} as Block;

    const [code, order] = get_sound_direction(block, generator);

    expect(code).toBe("get_sound_direction()");
    expect(order).toBe(Order.FUNCTION_CALL);

    const defs = Object.values(generator.definitions_).join("\n");
    expect(defs).toContain("from std_msgs.msg import Int32");
    expect(defs).toContain('Int32, "/doa_angle"');
    expect(defs).toContain("node.create_subscription");
    expect(defs).toContain("rclpy.spin_once(node");
    expect(defs).toContain("node.destroy_subscription(subscription)");
    expect(defs).toContain("timeout_sec = 5.0");
    expect(defs).toContain("return int(received[\"data\"])");
    expect(defs).toContain("return 0");
  });
});
