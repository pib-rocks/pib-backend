import { Block } from "blockly/core/block";
import { Order, pythonGenerator } from "blockly/python";
import {
  imu_get_acceleration,
  imu_get_angular_velocity,
  imu_get_data_age,
  imu_is_data_available,
} from "../../pib_blockly/pib_blockly_server/src/pib-blockly/program-generators/imu-generators";

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

function blockWithAxis(axis: string): Block {
  return {
    getFieldValue: (name: string) => {
      if (name === "AXIS") return axis;
      throw new Error(`unexpected field ${name}`);
    },
  } as Block;
}

function expectDualHostImuFallback(defs: string, safeDefault: string) {
  expect(defs).toContain("from pib_sdk.features.imu import IMU");
  expect(defs).toContain('os.getenv("ROSBRIDGE_HOST", "rosbridge-ws")');
  expect(defs).toContain("with IMU(host=rosbridge_host, port=9090) as imu:");
  expect(defs).toContain("imu.latest()");
  expect(defs).toContain('IMU(host="localhost", port=9090)');
  expect(defs).toContain(`return ${safeDefault}`);
}

describe("imu_get_acceleration generator", () => {
  it("emits IMU acceleration via a rosbridge helper for an axis", () => {
    const generator = createMockGenerator();
    const [code, order] = imu_get_acceleration(blockWithAxis("x"), generator);

    expect(code).toBe('get_imu_acceleration("x")');
    expect(order).toBe(Order.FUNCTION_CALL);
    const defs = Object.values(generator.definitions_).join("\n");
    expectDualHostImuFallback(defs, "0.0");
    expect(defs).toContain("import math");
    expect(defs).toContain("data.acceleration_m_s2");
    expect(defs).toContain('if axis == "total":');
    expect(defs).toContain("math.sqrt(acc.x ** 2 + acc.y ** 2 + acc.z ** 2)");
  });

  it("passes the total axis into the helper", () => {
    const generator = createMockGenerator();
    const [code] = imu_get_acceleration(blockWithAxis("total"), generator);

    expect(code).toBe('get_imu_acceleration("total")');
  });

  it("rejects an invalid acceleration axis", () => {
    const generator = createMockGenerator();

    expect(() => imu_get_acceleration(blockWithAxis("w"), generator)).toThrow(
      "'w' is not a valid value for 'AXIS'.",
    );
  });
});

describe("imu_get_angular_velocity generator", () => {
  it("emits IMU angular velocity via a rosbridge helper", () => {
    const generator = createMockGenerator();
    const [code, order] = imu_get_angular_velocity(
      blockWithAxis("z"),
      generator,
    );

    expect(code).toBe('get_imu_angular_velocity("z")');
    expect(order).toBe(Order.FUNCTION_CALL);
    const defs = Object.values(generator.definitions_).join("\n");
    expectDualHostImuFallback(defs, "0.0");
    expect(defs).toContain("data.angular_velocity_rad_s");
    expect(defs).not.toContain("import math");
  });

  it("rejects total as an angular velocity axis", () => {
    const generator = createMockGenerator();

    expect(() =>
      imu_get_angular_velocity(blockWithAxis("total"), generator),
    ).toThrow("'total' is not a valid value for 'AXIS'.");
  });
});

describe("imu_is_data_available generator", () => {
  it("emits IMU availability via a rosbridge helper", () => {
    const generator = createMockGenerator();
    const [code, order] = imu_is_data_available({} as Block, generator);

    expect(code).toBe("get_imu_is_data_available()");
    expect(order).toBe(Order.FUNCTION_CALL);
    const defs = Object.values(generator.definitions_).join("\n");
    expectDualHostImuFallback(defs, "False");
    expect(defs).toContain("return imu.latest() is not None");
  });
});

describe("imu_get_data_age generator", () => {
  it("emits IMU data age via a rosbridge helper", () => {
    const generator = createMockGenerator();
    const [code, order] = imu_get_data_age({} as Block, generator);

    expect(code).toBe("get_imu_data_age()");
    expect(order).toBe(Order.FUNCTION_CALL);
    const defs = Object.values(generator.definitions_).join("\n");
    expectDualHostImuFallback(defs, "0.0");
    expect(defs).toContain("return data.age_s");
  });
});
