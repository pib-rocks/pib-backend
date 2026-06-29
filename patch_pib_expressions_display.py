#!/usr/bin/env python3
from pathlib import Path
import re

ROOT = Path("/root/app/pib-backend")
CEREBRA = Path("/root/app/cerebra")

DISPLAY_PKG = ROOT / "ros_packages/display"
DISPLAY_MANAGER = DISPLAY_PKG / "display/expression_manager.py"
DISPLAY_SETUP = DISPLAY_PKG / "setup.py"
DISPLAY_PACKAGE_XML = DISPLAY_PKG / "package.xml"
COMPOSE = ROOT / "docker-compose.yaml"

BLOCKLY_BACKEND = ROOT / "pib_blockly/pib_blockly_server/src/pib-blockly"
BLOCKLY_CEREBRA = CEREBRA / "src/app/program/pib-blockly"
TOOLBOX = CEREBRA / "src/app/program/blockly.ts"

EXPRESSION_MANAGER_PY = r'''import os
import re
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datatypes.msg import DisplayImage


class PibExpressionManager(Node):
    def __init__(self):
        super().__init__("pib_expression_manager")

        self.expression_dir = Path(
            os.environ.get("PIB_EXPRESSION_DIR", "/app/pib-expression-faces")
        )

        self.publisher = self.create_publisher(DisplayImage, "/display_image", 10)
        self.subscription = self.create_subscription(
            String,
            "/pib/expression",
            self.on_expression,
            10,
        )

        self.get_logger().info("PIB expression manager started")
        self.get_logger().info(f"Expression directory: {self.expression_dir}")

    def normalize_expression(self, value: str) -> str:
        value = value.strip().lower()
        value = value.replace("-", "_").replace(" ", "_")
        value = re.sub(r"[^a-z0-9_]", "", value)
        return value

    def find_expression_file(self, expression: str) -> Path:
        for suffix in (".png", ".gif", ".jpg", ".jpeg"):
            candidate = self.expression_dir / f"{expression}{suffix}"
            if candidate.exists():
                return candidate

        raise FileNotFoundError(
            f"Expression file not found for '{expression}' in {self.expression_dir}. "
            f"Expected {expression}.png, {expression}.gif, {expression}.jpg or {expression}.jpeg"
        )

    def get_image_format(self, path: Path) -> int:
        suffix = path.suffix.lower()

        if suffix == ".gif":
            return 0  # ANIMATED_GIF
        if suffix == ".png":
            return 1  # PNG
        if suffix in (".jpg", ".jpeg"):
            return 2  # JPEG

        raise RuntimeError(f"Unsupported image format: {path}")

    def publish_expression_file(self, path: Path):
        msg = DisplayImage()
        msg.id.value = 1  # CUSTOM
        msg.format.value = self.get_image_format(path)
        msg.data = list(path.read_bytes())

        self.publisher.publish(msg)

    def on_expression(self, msg: String):
        expression = self.normalize_expression(msg.data)

        try:
            path = self.find_expression_file(expression)
            self.publish_expression_file(path)
            self.get_logger().info(f"Expression shown: {expression} -> {path.name}")
        except Exception as exc:
            self.get_logger().error(str(exc))


def main(args=None):
    rclpy.init(args=args)
    node = PibExpressionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
'''

DISPLAY_BLOCKS_TS = '''import * as Blockly from "blockly";

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
    tooltip: "Shows an expression on Pib's face display.",
    helpUrl: "",
  },
]);
'''

DISPLAY_GENERATORS_TS = '''import { Block } from "blockly/core/block";
import { pythonGenerator } from "blockly/python";

function runtime() {
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

  return `${runtime()}
_msg = String()
_msg.data = "${expression}"
_pib_expression_pub.publish(_msg)
rclpy.spin_once(_pib_expression_node, timeout_sec=0.1)
time.sleep(0.1)
`;
}

export { pythonGenerator };
'''

def patch_setup_py():
    text = DISPLAY_SETUP.read_text()

    if '"expression_manager = display.expression_manager:main",' not in text:
        text = text.replace(
            '"display = display.display:main",',
            '"display = display.display:main",\n            "expression_manager = display.expression_manager:main",',
        )

    DISPLAY_SETUP.write_text(text)
    print(f"patched {DISPLAY_SETUP}")

def patch_package_xml():
    text = DISPLAY_PACKAGE_XML.read_text()

    for dep in [
        "  <exec_depend>std_msgs</exec_depend>",
        "  <exec_depend>datatypes</exec_depend>",
    ]:
        if dep not in text:
            text = text.replace("  <exec_depend>rclpy</exec_depend>", "  <exec_depend>rclpy</exec_depend>\n" + dep)

    DISPLAY_PACKAGE_XML.write_text(text)
    print(f"patched {DISPLAY_PACKAGE_XML}")

def patch_compose():
    text = COMPOSE.read_text()

    if "pib-expression-manager:" in text:
        print("compose already contains pib-expression-manager")
        return

    service = '''  pib-expression-manager:
    image: ros-display
    privileged: true
    build:
      context: .
      dockerfile: ./ros_packages/display/Dockerfile
    profiles:
      - all
      - display
    volumes:
      - /root/app/pib-expression-faces:/app/pib-expression-faces:ro
    environment:
      - PIB_EXPRESSION_DIR=/app/pib-expression-faces
    command: ros2 run display expression_manager
    restart: always
    networks:
      - pib-network
    depends_on:
      - ros-display
'''

    marker = "\n  ros-audio-io:"
    if marker not in text:
        raise SystemExit("Could not find ros-audio-io marker in docker-compose.yaml")

    text = text.replace(marker, "\n" + service + marker, 1)
    COMPOSE.write_text(text)
    print(f"patched {COMPOSE}")

def patch_blockly_target(root: Path):
    if not root.exists():
        print(f"skip missing {root}")
        return

    blocks = root / "program-blocks/display-blocks.ts"
    generators = root / "program-generators/display-generators.ts"
    custom_blocks = root / "program-blocks/custom-blocks.ts"
    custom_generators = root / "program-generators/custom-generators.ts"

    blocks.write_text(DISPLAY_BLOCKS_TS)
    generators.write_text(DISPLAY_GENERATORS_TS)

    cb = custom_blocks.read_text()
    if 'import { displayBlocks } from "./display-blocks";' not in cb:
        lines = cb.splitlines()
        last_import = max(i for i, line in enumerate(lines) if line.startswith("import "))
        lines.insert(last_import + 1, 'import { displayBlocks } from "./display-blocks";')
        cb = "\n".join(lines) + "\n"

    if "Blockly.common.defineBlocks(displayBlocks);" not in cb:
        cb = cb.replace(
            "export function customBlockDefinition() {",
            "export function customBlockDefinition() {\n  Blockly.common.defineBlocks(displayBlocks);",
            1,
        )

    custom_blocks.write_text(cb)

    cg = custom_generators.read_text()
    if 'import * as displayGenerators from "./display-generators";' not in cg:
        lines = cg.splitlines()
        last_import = max(i for i, line in enumerate(lines) if line.startswith("import "))
        lines.insert(last_import + 1, 'import * as displayGenerators from "./display-generators";')
        cg = "\n".join(lines) + "\n"

    if "...displayGenerators," not in cg:
        cg = cg.replace("    ...tfButton,", "    ...tfButton,\n    ...displayGenerators,", 1)

    if 'pythonGenerator.forBlock["set_face_expression"]' not in cg:
        cg = cg.rstrip() + '''

pythonGenerator.forBlock["set_face_expression"] =
  generators["setFaceExpressionGenerator"];
'''

    custom_generators.write_text(cg)

    print(f"patched blockly target {root}")

def patch_toolbox():
    if not TOOLBOX.exists():
        print(f"skip missing toolbox {TOOLBOX}")
        return

    text = TOOLBOX.read_text()

    if 'category name="Expressions"' in text:
        print("toolbox already contains Expressions")
        return

    category = '''<category name="Expressions" colour="180">
    <block type="set_face_expression"></block>
</category>
'''

    marker = '<category name="Buttons" colour="20">'
    if marker not in text:
        raise SystemExit("Could not find Buttons category in blockly.ts")

    text = text.replace(marker, category + marker, 1)
    TOOLBOX.write_text(text)
    print(f"patched {TOOLBOX}")

DISPLAY_MANAGER.write_text(EXPRESSION_MANAGER_PY)
print(f"written {DISPLAY_MANAGER}")

patch_setup_py()
patch_package_xml()
patch_compose()
patch_blockly_target(BLOCKLY_BACKEND)
patch_blockly_target(BLOCKLY_CEREBRA)
patch_toolbox()

Path("/root/app/pib-expression-faces").mkdir(parents=True, exist_ok=True)

print("""
DONE.

Next:
1) Copy expression files into /root/app/pib-expression-faces
2) Build/restart:
   cd /root/app/pib-backend
   docker compose --profile display up -d --build ros-display pib-expression-manager

3) Test:
   docker exec -it pib-backend-ros-display-1 bash -lc 'source /opt/ros/humble/setup.bash && source /app/ros2_ws/install/setup.bash && ros2 topic pub --once /pib/expression std_msgs/msg/String "{data: happy}"'
""")
