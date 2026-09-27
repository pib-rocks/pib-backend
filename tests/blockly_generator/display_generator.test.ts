import { Block } from "blockly/core/block";
import {
  closeCerebraFullscreenGenerator,
  openCerebraFullscreenGenerator,
} from "../../pib_blockly/pib_blockly_server/src/pib-blockly/program-generators/display-generators";
import { displayBlocks } from "../../pib_blockly/pib_blockly_server/src/pib-blockly/program-blocks/display-blocks";
import { pythonGenerator } from "../../pib_blockly/pib_blockly_server/src/pib-blockly/program-generators/custom-generators";

const OPEN_TOPIC = "/pib/display_web";
const HIDE_TOPIC = "/pib/display_web_hide";

function urlBlock(url: string): Block {
  return {
    getFieldValue: (name: string) => (name === "URL" ? url : null),
  } as unknown as Block;
}

// Every `_pib_publish_string(...)` call in the generated program text. The
// runtime preamble defines the function but never calls it, so this is the
// list of what the block actually publishes.
function publishCalls(code: string): string[] {
  return code
    .split("\n")
    .filter((line) => line.startsWith("_pib_publish_string("));
}

describe("open_cerebra_fullscreen generator", () => {
  it("publishes the URL from the field to /pib/display_web", () => {
    const code = openCerebraFullscreenGenerator(urlBlock("http://localhost"));
    expect(code).toContain(
      `_pib_display_web_pub = _pib_display_node.create_publisher(String, "${OPEN_TOPIC}", 10)`,
    );
    expect(code).toContain(
      '_pib_publish_string(_pib_display_web_pub, "http://localhost", "display web page")',
    );
    expect(code).toContain("_pib_wait_for_subscriber");
  });

  it("changes the generated text when the URL field changes", () => {
    const first = openCerebraFullscreenGenerator(urlBlock("http://localhost"));
    const second = openCerebraFullscreenGenerator(
      urlBlock("http://localhost:8080/program"),
    );
    expect(first).not.toEqual(second);
    expect(second).toContain('"http://localhost:8080/program"');
    expect(second).not.toContain('"http://localhost"');
  });

  it("emits the URL as a JSON string literal, not by concatenation", () => {
    const tricky = 'http://localhost/?q="a"\\b';
    const code = openCerebraFullscreenGenerator(urlBlock(tricky));
    expect(code).toContain(
      `_pib_publish_string(_pib_display_web_pub, ${JSON.stringify(tricky)}, "display web page")`,
    );
  });

  it("falls back to http://localhost when the field is empty", () => {
    const code = openCerebraFullscreenGenerator(urlBlock("   "));
    expect(code).toContain(
      '_pib_publish_string(_pib_display_web_pub, "http://localhost", "display web page")',
    );
  });

  it("publishes exactly once, to the web publisher only", () => {
    const code = openCerebraFullscreenGenerator(urlBlock("http://localhost"));
    expect(publishCalls(code)).toEqual([
      '_pib_publish_string(_pib_display_web_pub, "http://localhost", "display web page")',
    ]);
  });
});

describe("close_cerebra_fullscreen generator", () => {
  it("publishes to /pib/display_web_hide", () => {
    const code = closeCerebraFullscreenGenerator({} as Block);
    expect(code).toContain(
      `_pib_display_web_hide_pub = _pib_display_node.create_publisher(String, "${HIDE_TOPIC}", 10)`,
    );
    expect(code).toContain(
      '_pib_publish_string(_pib_display_web_hide_pub, "hide", "display web hide")',
    );
    expect(code).toContain("_pib_wait_for_subscriber");
  });

  it("publishes exactly once, to the hide publisher only", () => {
    const code = closeCerebraFullscreenGenerator({} as Block);
    expect(publishCalls(code)).toEqual([
      '_pib_publish_string(_pib_display_web_hide_pub, "hide", "display web hide")',
    ]);
  });
});

describe("cerebra fullscreen block definitions and registration", () => {
  it("defines exactly the four display blocks; the old toggle block is gone", () => {
    expect(Object.keys(displayBlocks).sort()).toEqual([
      "close_cerebra_fullscreen",
      "open_cerebra_fullscreen",
      "set_face_expression",
      "show_face_text",
    ]);
  });

  it("defines both blocks in the Expressions colour with a URL field on open", () => {
    const open = displayBlocks["open_cerebra_fullscreen"];
    const close = displayBlocks["close_cerebra_fullscreen"];
    expect(open).toBeDefined();
    expect(close).toBeDefined();

    const json = (definitionJson(open) as { colour: number; args0: unknown[] });
    expect(json.colour).toBe(180);
    expect(json.args0).toEqual([
      expect.objectContaining({
        type: "field_input",
        name: "URL",
        text: "http://localhost",
      }),
    ]);
    expect((definitionJson(close) as { colour: number }).colour).toBe(180);
  });

  it("registers the python generators under the block type names", () => {
    expect(pythonGenerator.forBlock["open_cerebra_fullscreen"]).toBe(
      openCerebraFullscreenGenerator,
    );
    expect(pythonGenerator.forBlock["close_cerebra_fullscreen"]).toBe(
      closeCerebraFullscreenGenerator,
    );
  });

  it("registers no generator for a block type that is not defined", () => {
    // Every registered cerebra_* generator must belong to a defined block, so
    // a stale registration for a deleted block type cannot survive unnoticed.
    const registered = Object.keys(pythonGenerator.forBlock).filter((name) =>
      name.includes("cerebra"),
    );
    expect(registered.sort()).toEqual([
      "close_cerebra_fullscreen",
      "open_cerebra_fullscreen",
    ]);
    for (const name of registered) {
      expect(displayBlocks[name]).toBeDefined();
    }
  });
});

// createBlockDefinitionsFromJsonArray wraps each JSON definition in an init()
// that calls this.jsonInit(json). Capture the json without a workspace.
function definitionJson(definition: { init: () => void }): unknown {
  let captured: unknown = null;
  definition.init.call({
    jsonInit(json: unknown) {
      captured = json;
    },
  });
  return captured;
}
