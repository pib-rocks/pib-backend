import { Block } from "blockly/core/block";
import { Order, pythonGenerator } from "blockly/python";
import {
  camera_get_depth_frame,
  camera_get_distance_at_px,
} from "../../pib_blockly/pib_blockly_server/src/pib-blockly/program-generators/camera-generators";

type MockGenerator = typeof pythonGenerator & {
  definitions_: Record<string, string>;
  provideFunction_: (name: string, code: string) => string;
  valueToCode: (
    block: Block,
    name: string,
    order: Order,
  ) => string | [string, Order];
};

function createMockGenerator(): MockGenerator {
  const definitions: Record<string, string> = {};
  const generator = Object.create(pythonGenerator) as MockGenerator;
  generator.definitions_ = definitions;
  generator.provideFunction_ = (name: string, code: string) => {
    definitions[`FN_${name}`] = code;
    return name;
  };
  generator.valueToCode = () => "0";
  return generator;
}

describe("camera_get_depth_frame generator", () => {
  it("emits Camera.get_depth_frame via a rosbridge helper", () => {
    const generator = createMockGenerator();
    const block = {} as Block;

    const [code, order] = camera_get_depth_frame(block, generator);

    expect(code).toBe("get_camera_depth_frame()");
    expect(order).toBe(Order.FUNCTION_CALL);
    const defs = Object.values(generator.definitions_).join("\n");
    expect(defs).toContain("from pib_sdk.features.camera import Camera");
    expect(defs).toContain('os.getenv("ROSBRIDGE_HOST", "rosbridge-ws")');
    expect(defs).toContain("with Camera(host=rosbridge_host, port=9090) as cam:");
    expect(defs).toContain("return cam.get_depth_frame()");
    expect(defs).toContain('Camera(host="localhost", port=9090)');
  });
});

describe("camera_get_distance_at_px generator", () => {
  it("emits Camera.get_distance_at_px with x and y inputs", () => {
    const generator = createMockGenerator();
    generator.valueToCode = (_block, name) => {
      if (name === "X") return "120";
      if (name === "Y") return "80";
      throw new Error(`unexpected input ${name}`);
    };
    const block = {} as Block;

    const [code, order] = camera_get_distance_at_px(block, generator);

    expect(code).toBe("get_camera_distance_at_px(120, 80)");
    expect(order).toBe(Order.FUNCTION_CALL);
    const defs = Object.values(generator.definitions_).join("\n");
    expect(defs).toContain("from pib_sdk.features.camera import Camera");
    expect(defs).toContain('os.getenv("ROSBRIDGE_HOST", "rosbridge-ws")');
    expect(defs).toContain("with Camera(host=rosbridge_host, port=9090) as cam:");
    expect(defs).toContain("return cam.get_distance_at_px(int(x), int(y))");
    expect(defs).toContain('Camera(host="localhost", port=9090)');
  });

  it("defaults missing x/y inputs to 0", () => {
    const generator = createMockGenerator();
    generator.valueToCode = () => "";
    const block = {} as Block;

    const [code] = camera_get_distance_at_px(block, generator);

    expect(code).toBe("get_camera_distance_at_px(0, 0)");
  });
});
