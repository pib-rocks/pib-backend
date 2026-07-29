import { Block } from "blockly/core/block";
import { toggleCerebraFullscreenGenerator } from "../../pib_blockly/pib_blockly_server/src/pib-blockly/program-generators/display-generators";

describe("toggle_cerebra_fullscreen generator", () => {
  it("generates ROS display toggle command", () => {
    const block = {} as Block;
    const code = toggleCerebraFullscreenGenerator(block);
    expect(code).toContain("TOGGLE_CEREBRA_FULLSCREEN");
    expect(code).toContain("_pib_expression_pub");
  });
});
