import * as Blockly from "blockly";

const DEFAULT_HOST = "localhost";
const DEFAULT_USER = "pib";
const DEFAULT_PASSWORD = "pib";
const DEFAULT_PORT = 22;

// Mixin that adds the "gear" mutator used to optionally expose the SSH
// connection settings (host / user / password / port) on the block.
const CONNECTION_MUTATOR_MIXIN = {
    showSettings_: false,
    host_: DEFAULT_HOST,
    user_: DEFAULT_USER,
    password_: DEFAULT_PASSWORD,
    port_: DEFAULT_PORT,

    saveExtraState: function (this: any) {
        // make sure the cached values reflect the currently shown fields
        if (this.getInput("CONNECTION")) {
            this.host_ = this.getFieldValue("HOST") ?? this.host_;
            this.user_ = this.getFieldValue("USER") ?? this.user_;
            this.password_ = this.getFieldValue("PASSWORD") ?? this.password_;
            this.port_ = Number(this.getFieldValue("PORT") ?? this.port_);
        }
        return {
            showSettings: this.showSettings_,
            host: this.host_,
            user: this.user_,
            password: this.password_,
            port: this.port_,
        };
    },

    loadExtraState: function (this: any, state: any) {
        this.showSettings_ = !!state["showSettings"];
        this.host_ = state["host"] ?? DEFAULT_HOST;
        this.user_ = state["user"] ?? DEFAULT_USER;
        this.password_ = state["password"] ?? DEFAULT_PASSWORD;
        this.port_ = state["port"] ?? DEFAULT_PORT;
        this.updateShape_();
    },

    decompose: function (this: any, workspace: Blockly.Workspace) {
        const containerBlock = workspace.newBlock(
            "run_script_mutator_container",
        );
        (containerBlock as Blockly.BlockSvg).initSvg();
        if (this.showSettings_) {
            const itemBlock = workspace.newBlock("run_script_mutator_item");
            (itemBlock as Blockly.BlockSvg).initSvg();
            containerBlock
                .getInput("STACK")!
                .connection!.connect(itemBlock.previousConnection!);
        }
        return containerBlock as Blockly.BlockSvg;
    },

    compose: function (this: any, containerBlock: Blockly.Block) {
        const itemBlock = containerBlock.getInputTargetBlock("STACK");
        this.showSettings_ = !!itemBlock;
        this.updateShape_();
    },

    updateShape_: function (this: any) {
        const hasInput = this.getInput("CONNECTION");
        if (this.showSettings_ && !hasInput) {
            this.appendDummyInput("CONNECTION")
                .appendField("host")
                .appendField(
                    new Blockly.FieldTextInput(this.host_, (value: string) => {
                        this.host_ = value;
                        return value;
                    }),
                    "HOST",
                )
                .appendField("user")
                .appendField(
                    new Blockly.FieldTextInput(this.user_, (value: string) => {
                        this.user_ = value;
                        return value;
                    }),
                    "USER",
                )
                .appendField("password")
                .appendField(
                    new Blockly.FieldTextInput(
                        this.password_,
                        (value: string) => {
                            this.password_ = value;
                            return value;
                        },
                    ),
                    "PASSWORD",
                )
                .appendField("port")
                .appendField(
                    new Blockly.FieldNumber(
                        this.port_,
                        1,
                        65535,
                        1,
                        (value: string | number) => {
                            this.port_ = Number(value);
                            return Number(value);
                        },
                    ),
                    "PORT",
                );
        } else if (!this.showSettings_ && hasInput) {
            // cache the current values before the fields disappear
            this.host_ = this.getFieldValue("HOST") ?? this.host_;
            this.user_ = this.getFieldValue("USER") ?? this.user_;
            this.password_ = this.getFieldValue("PASSWORD") ?? this.password_;
            this.port_ = Number(this.getFieldValue("PORT") ?? this.port_);
            this.removeInput("CONNECTION");
        }
    },
};

if (
    !(Blockly.Extensions as any).isRegistered?.("run_script_connection_mutator")
) {
    try {
        Blockly.Extensions.registerMutator(
            "run_script_connection_mutator",
            CONNECTION_MUTATOR_MIXIN as any,
            undefined,
            ["run_script_mutator_item"],
        );
    } catch (e) {
        // already registered (e.g. on hot-module-reload) - safe to ignore
    }
}

export const runScriptBlocks =
    Blockly.common.createBlockDefinitionsFromJsonArray([
        {
            type: "run_script",
            message0: "Run script %1",
            args0: [
                {
                    type: "input_value",
                    name: "SCRIPT",
                    check: "String",
                },
            ],
            previousStatement: null,
            nextStatement: null,
            colour: 45,
            tooltip:
                "Connects via SSH to a host and runs the given script. Default host localhost reaches this machine; in Docker it is resolved automatically via SSH_HOST. Use the gear to edit host/user/password/port.",
            helpUrl: "",
            mutator: "run_script_connection_mutator",
        },
        {
            type: "run_script_mutator_container",
            message0: "connection settings %1 %2",
            args0: [
                {type: "input_dummy"},
                {type: "input_statement", name: "STACK"},
            ],
            colour: 45,
            tooltip:
                "Drag the item below in to use custom connection settings.",
            helpUrl: "",
        },
        {
            type: "run_script_mutator_item",
            message0: "use custom connection",
            previousStatement: null,
            nextStatement: null,
            colour: 45,
            tooltip:
                "When present, host/user/password/port fields are shown on the block.",
            helpUrl: "",
        },
    ]);
