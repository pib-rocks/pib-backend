import * as Blockly from "blockly";
import { Block } from "blockly/core/block";
import { Order, pythonGenerator } from "blockly/python";
import {
  program_log,
  program_reset_log,
} from "../../pib_blockly/pib_blockly_server/src/pib-blockly/program-generators/program-log-generator";
import { programLogBlocks } from "../../pib_blockly/pib_blockly_server/src/pib-blockly/program-blocks/program-log-blocks";

type MockGenerator = typeof pythonGenerator & {
  definitions_: Record<string, string>;
  provideFunction_: (name: string, code: string) => string;
  valueToCode: (
    block: Block,
    name: string,
    order: Order,
  ) => string | [string, Order];
};

function createMockGenerator(textCode = '"hello"'): MockGenerator {
  const definitions: Record<string, string> = {};
  const generator = Object.create(pythonGenerator) as MockGenerator;
  generator.definitions_ = definitions;
  generator.provideFunction_ = (name: string, code: string) => {
    definitions[`FN_${name}`] = code;
    return name;
  };
  generator.valueToCode = () => textCode;
  return generator;
}

function createLogBlock(level: string): Block {
  return {
    getFieldValue: (field: string) => {
      if (field === "LEVEL") return level;
      throw new Error(`unexpected field ${field}`);
    },
  } as unknown as Block;
}

function defs(generator: MockGenerator) {
  return Object.values(generator.definitions_).join("\n");
}

describe("program_log generator", () => {
  it("appends a timestamped INFO line to the program log file", () => {
    const generator = createMockGenerator('"hello"');
    const code = program_log(createLogBlock("INFO"), generator);

    expect(code).toBe('program_log("INFO", "hello")\n');
    const generated = defs(generator);
    expect(generated).toContain("import os");
    expect(generated).toContain("import datetime");
    expect(generated).toContain('os.getenv("PROGRAM_DIR", "/home/pib/cerebra_programs")');
    expect(generated).toContain("program-logs");
    expect(generated).toContain('os.path.splitext(os.path.basename(__file__))[0]');
    expect(generated).toContain("os.makedirs(log_dir, exist_ok=True)");
    expect(generated).toContain('open(program_log_path(), "a", encoding="utf-8")');
    expect(generated).toContain(
      'log_file.write(f"[{datetime.datetime.now()}] {level} {text}\\n")',
    );
  });

  it("expands the selected severity into the helper call", () => {
    const generator = createMockGenerator("message");
    const code = program_log(createLogBlock("WARNING"), generator);

    expect(code).toBe('program_log("WARNING", message)\n');
  });

  it("defaults a missing text input to an empty string", () => {
    const generator = createMockGenerator("");
    const code = program_log(createLogBlock("ERROR"), generator);

    expect(code).toBe('program_log("ERROR", "")\n');
  });

  it("rejects an unknown log level", () => {
    const generator = createMockGenerator();
    expect(() => program_log(createLogBlock("TRACE"), generator)).toThrow(
      "'TRACE' is not a valid value for 'LEVEL'.",
    );
  });
});

describe("program_reset_log generator", () => {
  it("truncates the same program log file", () => {
    const generator = createMockGenerator();
    const code = program_reset_log({} as Block, generator);

    expect(code).toBe("program_reset_log()\n");
    const generated = defs(generator);
    expect(generated).toContain("import os");
    expect(generated).toContain('os.getenv("PROGRAM_DIR", "/home/pib/cerebra_programs")');
    expect(generated).toContain("program-logs");
    expect(generated).toContain("os.makedirs(log_dir, exist_ok=True)");
    expect(generated).toContain('open(program_log_path(), "w", encoding="utf-8")');
    expect(generated).toContain('log_file.write("")');
  });
});

describe("program log blocks", () => {
  beforeAll(() => Blockly.common.defineBlocks(programLogBlocks));

  it("defines a log statement with INFO default and a TEXT value input", () => {
    const workspace = new Blockly.Workspace();
    Blockly.Events.disable();
    try {
      const block = workspace.newBlock("program_log");
      expect(block.getFieldValue("LEVEL")).toBe("INFO");
      expect(block.getField("LEVEL")).toBeInstanceOf(Blockly.FieldDropdown);
      expect(block.getInput("TEXT")).toBeTruthy();
      expect(block.previousConnection).toBeTruthy();
      expect(block.nextConnection).toBeTruthy();
    } finally {
      workspace.dispose();
      Blockly.Events.enable();
    }
  });

  it("defines a reset-log statement with no inputs", () => {
    const workspace = new Blockly.Workspace();
    Blockly.Events.disable();
    try {
      const block = workspace.newBlock("program_reset_log");
      expect(block.getField("LEVEL")).toBeNull();
      expect(block.getInput("TEXT")).toBeFalsy();
      expect(block.previousConnection).toBeTruthy();
      expect(block.nextConnection).toBeTruthy();
    } finally {
      workspace.dispose();
      Blockly.Events.enable();
    }
  });
});
