import * as Blockly from "blockly";
import {MOTOR_NAME_DROPDOWN_OPTIONS} from "./motor-motor-options";

const MOVE_MOTORS_TOOLTIP =
    "Moves the selected motors in one joint trajectory. Each target angle is in degrees. Typical limits are about ±90° per joint (see motor configuration). Values are passed to the robot in degrees.";

type MoveMotorsBlock = Blockly.Block & {
    itemCount_: number;
    updateShape_: () => void;
};

function updateMoveMotorsShape(block: MoveMotorsBlock) {
    for (let i = 0; block.getInput(`ANGLE_${i}`); i++) {
        block.removeInput(`ROW_${i}`);
        block.removeInput(`ANGLE_${i}`);
    }

    for (let i = 0; i < block.itemCount_; i++) {
        const row = block
            .appendDummyInput(`ROW_${i}`)
            .appendField(i === 0 ? "move motors" : "and motor")
            .appendField(
                new Blockly.FieldDropdown(MOTOR_NAME_DROPDOWN_OPTIONS),
                `MOTOR_${i}`,
            )
            .appendField("to position in degrees");
        if (i === 0) {
            row.appendField("from input");
        }
        block
            .appendValueInput(`ANGLE_${i}`)
            .setCheck("Number")
            .setAlign(Blockly.inputs.Align.RIGHT);
    }
}

const MOVE_MOTORS_MUTATOR_MIXIN = {
    saveExtraState: function (this: MoveMotorsBlock) {
        return {itemCount: this.itemCount_};
    },

    loadExtraState: function (
        this: MoveMotorsBlock,
        state: {itemCount?: number},
    ) {
        this.itemCount_ = state.itemCount ?? 2;
        if (this.itemCount_ < 1) {
            this.itemCount_ = 1;
        }
        this.updateShape_();
    },

    decompose: function (this: MoveMotorsBlock, workspace: Blockly.Workspace) {
        const container = workspace.newBlock("move_motors_mutator_container");
        (container as Blockly.BlockSvg).initSvg();
        let connection = container.getInput("STACK")!.connection!;
        for (let i = 0; i < this.itemCount_; i++) {
            const item = workspace.newBlock("move_motors_mutator_item");
            (item as Blockly.BlockSvg).initSvg();
            connection.connect(item.previousConnection!);
            connection = item.nextConnection!;
        }
        return container as Blockly.BlockSvg;
    },

    compose: function (this: MoveMotorsBlock, containerBlock: Blockly.Block) {
        let itemBlock = containerBlock.getInputTargetBlock("STACK");
        const valueConnections: Blockly.Connection[] = [];
        while (itemBlock) {
            if (!itemBlock.isInsertionMarker()) {
                valueConnections.push(
                    (itemBlock as any).valueConnection_ as Blockly.Connection,
                );
            }
            itemBlock = itemBlock.getNextBlock();
        }

        for (let i = 0; i < this.itemCount_; i++) {
            const input = this.getInput(`ANGLE_${i}`);
            const target = input?.connection?.targetConnection;
            if (target && !valueConnections.includes(target)) {
                target.disconnect();
            }
        }

        this.itemCount_ = valueConnections.length || 1;
        this.updateShape_();

        for (let i = 0; i < this.itemCount_; i++) {
            valueConnections[i]?.reconnect(this, `ANGLE_${i}`);
        }
    },

    saveConnections: function (
        this: MoveMotorsBlock,
        containerBlock: Blockly.Block,
    ) {
        let itemBlock = containerBlock.getInputTargetBlock("STACK");
        for (let i = 0; itemBlock; i++) {
            if (itemBlock.isInsertionMarker()) {
                itemBlock = itemBlock.getNextBlock();
                continue;
            }
            const input = this.getInput(`ANGLE_${i}`);
            (itemBlock as any).valueConnection_ =
                input?.connection?.targetConnection ?? null;
            itemBlock = itemBlock.getNextBlock();
        }
    },
};

if (
    !(Blockly.Extensions as any).isRegistered?.("move_motors_row_mutator")
) {
    try {
        Blockly.Extensions.registerMutator(
            "move_motors_row_mutator",
            MOVE_MOTORS_MUTATOR_MIXIN as any,
            function (this: MoveMotorsBlock) {
                this.updateShape_();
            },
            ["move_motors_mutator_item"],
        );
    } catch {
        // already registered (e.g. on hot-module-reload) - safe to ignore
    }
}

const moveMotorsDefinition = {
    init: function (this: MoveMotorsBlock) {
        this.itemCount_ = 2;
        this.setColour(355);
        this.setTooltip(MOVE_MOTORS_TOOLTIP);
        this.setHelpUrl("");
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.updateShape_ = () => updateMoveMotorsShape(this);
        this.updateShape_();
        Blockly.Extensions.apply("move_motors_row_mutator", this, true);
    },
};

export const moveMotorsBlocks = {
    move_motors: moveMotorsDefinition,
    ...Blockly.common.createBlockDefinitionsFromJsonArray([
        {
            type: "move_motors_mutator_container",
            message0: "motors %1 %2",
            args0: [
                {type: "input_dummy"},
                {type: "input_statement", name: "STACK"},
            ],
            colour: 355,
            tooltip: "Add or remove motors with the blocks below.",
            helpUrl: "",
        },
        {
            type: "move_motors_mutator_item",
            message0: "motor row",
            previousStatement: null,
            nextStatement: null,
            colour: 355,
            tooltip: "Add another motor row to the block.",
            helpUrl: "",
        },
    ]),
};
