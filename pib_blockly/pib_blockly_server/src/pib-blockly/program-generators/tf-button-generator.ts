import * as Blockly from "blockly";
import {Block} from "blockly/core/block";
import {pythonGenerator} from "blockly/python";
import {
  CONFIGURE_LOGGING,
  IMPORT_LOGGING,
  IMPORT_RCLPY,
  IMPORT_SYS,
  IMPORT_TF_BUTTON_SERVICES,
  IMPORT_TF_BUTTON_BLOCKLY_CLIENT,
  INIT_ROS,
  INIT_TF_BUTTON_CLIENTS,
} from "./util/definitions";
import {
  TF_BUTTON_SET_COLOR_FUNCTION,
  TF_BUTTON_TASTER_FUNCTION,
  TF_BUTTON_SWITCH_FUNCTION,
} from "./util/function-declarations";

function hexToRgb(hex: string): {red: number; green: number; blue: number} {
  const normalized = hex.replace("#", "");
  return {
    red: parseInt(normalized.substring(0, 2), 16),
    green: parseInt(normalized.substring(2, 4), 16),
    blue: parseInt(normalized.substring(4, 6), 16),
  };
}

function configureGenerator(generator: typeof pythonGenerator): void {
  Object.assign(generator.definitions_, {
    IMPORT_RCLPY,
    IMPORT_SYS,
    IMPORT_LOGGING,
    IMPORT_TF_BUTTON_SERVICES,
    CONFIGURE_LOGGING,
    INIT_ROS,
    INIT_TF_BUTTON_CLIENTS,
  });
}

export function tf_button_taster_to_variable(
  block: Block,
  generator: typeof pythonGenerator,
) {
  const buttonId = block.getFieldValue("BUTTON_ID") || "1";
  const color = block.getFieldValue("COLOR") || "#00ff00";
  const variableId = block.getFieldValue("VAR");
  const variableName = generator.nameDB_.getName(
    variableId,
    "VARIABLE",
  );
  const {red, green, blue} = hexToRgb(color);

  configureGenerator(generator);

  const functionName = generator.provideFunction_(
    "tf_button_taster",
    TF_BUTTON_TASTER_FUNCTION(generator),
  );

  return `${variableName} = ${functionName}(${buttonId}, ${red}, ${green}, ${blue})\n`;
}

export function tf_button_switch_to_variable(
  block: Block,
  generator: typeof pythonGenerator,
) {
  const buttonId = block.getFieldValue("BUTTON_ID") || "1";
  const color = block.getFieldValue("COLOR") || "#ff0000";
  const variableId = block.getFieldValue("VAR");
  const variableName = generator.nameDB_.getName(
    variableId,
    "VARIABLE",
  );
  const {red, green, blue} = hexToRgb(color);

  configureGenerator(generator);

  const functionName = generator.provideFunction_(
    "tf_button_switch",
    TF_BUTTON_SWITCH_FUNCTION(generator),
  );

  return `${variableName} = ${functionName}(${buttonId}, ${red}, ${green}, ${blue})\n`;
}

export function tf_button_set_color(
  block: Block,
  generator: typeof pythonGenerator,
) {
  const buttonId = block.getFieldValue("BUTTON_ID") || "1";
  const color = block.getFieldValue("COLOR") || "#00ff00";
  const {red, green, blue} = hexToRgb(color);

  configureGenerator(generator);

  Object.assign(generator.definitions_, {
    IMPORT_TF_BUTTON_BLOCKLY_CLIENT,
  });

  return `blockly_client.set_button_color(${buttonId}, ${red}, ${green}, ${blue})\n`;
}

export {pythonGenerator};
