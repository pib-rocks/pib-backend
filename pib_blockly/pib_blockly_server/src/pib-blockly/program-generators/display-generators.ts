import { Block } from "blockly/core/block";
import { pythonGenerator } from "blockly/python";

function pibDisplayRuntime() {
  return `
import time
import rclpy
from std_msgs.msg import String

if not rclpy.ok():
    rclpy.init()

try:
    _pib_display_node
except NameError:
    _pib_display_node = rclpy.create_node("blockly_display_node")
    _pib_expression_pub = _pib_display_node.create_publisher(
        String,
        "/pib/expression",
        10
    )
    _pib_display_text_pub = _pib_display_node.create_publisher(
        String,
        "/pib/display_text",
        10
    )
    time.sleep(0.5)
`;
}

export function setFaceExpressionGenerator(block: Block) {
  const expression = block.getFieldValue("EXPRESSION");

  return `${pibDisplayRuntime()}
_msg = String()
_msg.data = "${expression}"

for _ in range(3):
    _pib_expression_pub.publish(_msg)
    rclpy.spin_once(_pib_display_node, timeout_sec=0.05)
    time.sleep(0.05)
`;
}

export function showFaceTextGenerator(block: Block) {
  const rawText = block.getFieldValue("TEXT") || "";
  const safeText = rawText.slice(0, 40).replace(/\\/g, "\\\\").replace(/"/g, "\\\"");

  return `${pibDisplayRuntime()}
_msg = String()
_msg.data = "${safeText}"[:40]

for _ in range(3):
    _pib_display_text_pub.publish(_msg)
    rclpy.spin_once(_pib_display_node, timeout_sec=0.05)
    time.sleep(0.05)
`;
}

export { pythonGenerator };
