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

export const poseBlocks =
    Blockly.common.createBlockDefinitionsFromJsonArray([
        {
            type: "save_current_pose",
            message0: "Save current pose as %1",
            args0: [
                {
                    type: "field_input",
                    name: "NAME",
                    text: "pose name",
                },
            ],
            previousStatement: null,
            nextStatement: null,
            colour: 355,
            tooltip:
                "Saves the last commanded position of every motor, including both head motors.",
            helpUrl: "",
        },
        {
            type: "get_all_poses",
            message0: "get all poses",
            output: "Array",
            colour: 355,
            tooltip: "Returns a list of all saved poses.",
            helpUrl: "",
        },
        {
            type: "get_pose_joints",
            message0: "get joints of pose %1",
            args0: [
                {
                    type: "field_input",
                    name: "NAME",
                    text: "pose name",
                },
            ],
            output: "Object",
            colour: 355,
            tooltip:
                "Returns the saved motor angles in degrees, keyed by motor name.",
            helpUrl: "",
        },
        {
            type: "has_pose",
            message0: "has pose %1",
            args0: [
                {
                    type: "field_input",
                    name: "NAME",
                    text: "pose name",
                },
            ],
            output: "Boolean",
            colour: 355,
            tooltip: "Returns whether a pose with this name exists.",
            helpUrl: "",
        },
        {
            type: "pose_count",
            message0: "pose count",
            output: "Number",
            colour: 355,
            tooltip: "Returns the number of saved poses.",
            helpUrl: "",
        },
    ]);

class CustomFieldDropdown extends Blockly.FieldDropdown {
    constructor(menuGenerator: any, opt_validator?: any, opt_config?: any) {
        super(menuGenerator, opt_validator, opt_config);
    }
    // Override the default validation function
    override doClassValidation_(newValue: any) {
        return newValue;
    }
}
