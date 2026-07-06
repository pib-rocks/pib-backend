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

def _pib_wait_for_subscriber(pub, label="display handler", timeout_sec=3.0):
    print(f"waiting for {label} to become available...")
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        rclpy.spin_once(_pib_display_node, timeout_sec=0.02)
        if pub.get_subscription_count() > 0:
            print(f"{label} available.")
            return True
        time.sleep(0.02)
    print(f"warning: {label} was not detected before timeout, trying anyway.")
    return False

def _pib_fix_text_encoding(value):
    try:
        return value.encode("latin1").decode("utf-8")
    except Exception:
        return value

def _pib_publish_string(pub, value, label="display"):
    if label == "text":
        value = _pib_fix_text_encoding(value)

    _pib_wait_for_subscriber(pub, label)
    _msg = String()
    _msg.data = value
    pub.publish(_msg)
    print(f"now displaying {label}: {value}")

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
_pib_publish_string(_pib_expression_pub, "${expression}", "expression")
`;
}

export function showFaceTextGenerator(block: Block) {
  const rawText = block.getFieldValue("TEXT") || "";
  const safeText = JSON.stringify(rawText.slice(0, 40));

  return `${pibDisplayRuntime()}
_pib_publish_string(_pib_display_text_pub, ${safeText}[:40], "text")
`;
}

export { pythonGenerator };
