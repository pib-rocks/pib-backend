import { Block } from "blockly/core/block";
import { pythonGenerator } from "blockly/python";
import { set_solid_state_relay } from "../../pib_blockly/pib_blockly_server/src/pib-blockly/program-generators/solid-state-relay-generator";

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

describe("set_solid_state_relay generator", () => {
  it("generates ON relay command with ROS client definitions", () => {
    const block = {
      getFieldValue: (field: string) => {
        if (field === "STATUS") return "ON";
        throw new Error(`unexpected field ${field}`);
      },
    } as unknown as Block;

    const generator = createMockGenerator();
    const code = set_solid_state_relay(block, generator);

    expect(code).toBe('set_solid_state_relay("ON")\n');
    const defs = Object.values(generator.definitions_).join("\n");
    expect(defs).toContain("set_solid_state_relay_state_client.call_async");
    expect(defs).toContain("SolidStateRelayState");
  });

  it("generates OFF relay command", () => {
    const block = {
      getFieldValue: (field: string) => {
        if (field === "STATUS") return "OFF";
        throw new Error(`unexpected field ${field}`);
      },
    } as unknown as Block;

    const generator = createMockGenerator();
    const code = set_solid_state_relay(block, generator);
    expect(code).toBe('set_solid_state_relay("OFF")\n');
  });
});
