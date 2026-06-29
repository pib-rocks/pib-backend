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
    _pib_expression_pub = _pib_display_node.create_publisher(String, "/pib/expression", 10)
    _pib_display_text_pub = _pib_display_node.create_publisher(String, "/pib/display_text", 10)

def _pib_wait_for_subscriber(pub, timeout_sec=3.0):
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        rclpy.spin_once(_pib_display_node, timeout_sec=0.02)
        if pub.get_subscription_count() > 0:
            return True
        time.sleep(0.02)
    return False

def _pib_publish_string(pub, value):
    _pib_wait_for_subscriber(pub)
    _msg = String()
    _msg.data = value
    pub.publish(_msg)

    # Keep the process alive briefly so DDS can deliver reliably.
    deadline = time.time() + 0.25
    while time.time() < deadline:
        rclpy.spin_once(_pib_display_node, timeout_sec=0.02)
        time.sleep(0.02)
`;
}

export function setFaceExpressionGenerator(block: Block) {
  const expression = block.getFieldValue("EXPRESSION");

  return `${pibDisplayRuntime()}
_pib_publish_string(_pib_expression_pub, "${expression}")
`;
}

export function showFaceTextGenerator(block: Block) {
  const rawText = block.getFieldValue("TEXT") || "";
  const safeText = rawText.slice(0, 40).replace(/\\/g, "\\\\").replace(/"/g, "\\\"");

  return `${pibDisplayRuntime()}
_pib_publish_string(_pib_display_text_pub, "${safeText}"[:40])
`;
}

export { pythonGenerator };
