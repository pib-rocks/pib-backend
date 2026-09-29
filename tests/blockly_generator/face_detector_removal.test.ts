import * as Blockly from "blockly";
import { pythonGenerator } from "blockly/python";
import { customBlockDefinition } from "../../pib_blockly/pib_blockly_server/src/pib-blockly/program-blocks/custom-blocks";
import "../../pib_blockly/pib_blockly_server/src/pib-blockly/program-generators/custom-generators";

describe("removed face detector blocks", () => {
  beforeAll(() => {
    customBlockDefinition();
  });

  it.each(["face_detector_start_stop", "face_detector_running"])(
    "does not register or generate %s",
    (blockType) => {
      expect(Blockly.Blocks[blockType]).toBeUndefined();
      expect(pythonGenerator.forBlock[blockType]).toBeUndefined();
    },
  );
});
