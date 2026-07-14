import * as Blockly from "blockly";

export const moveToPose = (Blockly.Blocks["move_to_pose"] = {
    init() {
        const input = this.appendDummyInput()
            .appendField("set Pose")
            .appendField(new CustomFieldDropdown(this.getPoses), "POSE");
        this.setColour(355);
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setTooltip("moves to selected pose");
        this.setHelpUrl("");
    },

    getPoses() {
        // Initial placeholder before options are fetched
        return [["Loading...", "LOADING"]];
    },
});
class CustomFieldDropdown extends Blockly.FieldDropdown {
    constructor(menuGenerator: any, opt_validator?: any, opt_config?: any) {
      super(menuGenerator, opt_validator, opt_config);
    }
    // Override the default validation function
    override doClassValidation_(newValue: any) {
      return newValue;
    }
}
