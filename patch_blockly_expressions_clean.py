#!/usr/bin/env python3
from pathlib import Path

TARGETS = [
    Path("/root/app/pib-backend/pib_blockly/pib_blockly_server/src/pib-blockly"),
    Path("/root/app/cerebra/src/app/program/pib-blockly"),
]

BLOCKS = '''import * as Blockly from "blockly";

export const displayBlocks = Blockly.common.createBlockDefinitionsFromJsonArray([
  {
    type: "set_face_expression",
    message0: "Set expression %1",
    args0: [
      {
        type: "field_dropdown",
        name: "EXPRESSION",
        options: [
          ["Happy", "happy"],
          ["Sad", "sad"],
          ["Sleep", "sleep"],
          ["Surprise", "surprise"],
          ["Thinking", "thinking"],
          ["Heart eyes", "heart_eyes"],
          ["Dizzy", "dizzy"],
          ["Angry", "angry"],
          ["Laugh", "laugh"],
          ["Cry", "cry"],
        ],
      },
    ],
    previousStatement: null,
    nextStatement: null,
    colour: 180,
    tooltip: "Shows an expression on Pib's face for 15 seconds, then returns to the default animated eyes.",
    helpUrl: "",
  },
]);
'''

GENERATORS = '''import { Block } from "blockly/core/block";
import { pythonGenerator } from "blockly/python";

function pibExpressionRuntime() {
  return `
import time
import rclpy
from std_msgs.msg import String

if not rclpy.ok():
    rclpy.init()

try:
    _pib_expression_node
except NameError:
    _pib_expression_node = rclpy.create_node("blockly_expression_node")
    _pib_expression_pub = _pib_expression_node.create_publisher(
        String,
        "/pib/expression",
        10
    )
    time.sleep(0.5)
`;
}

export function setFaceExpressionGenerator(block: Block) {
  const expression = block.getFieldValue("EXPRESSION");

  return `${pibExpressionRuntime()}
_msg = String()
_msg.data = "${expression}"
_pib_expression_pub.publish(_msg)
rclpy.spin_once(_pib_expression_node, timeout_sec=0.1)
time.sleep(0.1)
`;
}

export { pythonGenerator };
'''

for target in TARGETS:
    blocks = target / "program-blocks/display-blocks.ts"
    generators = target / "program-generators/display-generators.ts"

    blocks.write_text(BLOCKS)
    generators.write_text(GENERATORS)

    print(f"patched {blocks}")
    print(f"patched {generators}")

print("Blockly expression block patched.")
