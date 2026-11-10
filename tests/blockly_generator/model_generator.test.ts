import { execFileSync } from "node:child_process";
import { Block } from "blockly/core/block";
import { Order, pythonGenerator } from "blockly/python";
import {
  imitate_hands,
  start_model,
  stop_model,
} from "../../pib_blockly/pib_blockly_server/src/pib-blockly/program-generators/model-generators";
import {
  getModelDropdownOptions,
  getStopModelDropdownOptions,
  STOP_ALL_MODELS_VALUE,
} from "../../pib_blockly/pib_blockly_server/src/pib-blockly/program-blocks/model-blocks";

type MockGenerator = typeof pythonGenerator & {
  definitions_: Record<string, string>;
  provideFunction_: (name: string, code: string) => string;
  valueToCode: (
    block: Block,
    name: string,
    order: Order,
  ) => string;
};

function createMockGenerator(
  values: Record<string, string> = {},
): MockGenerator {
  const definitions: Record<string, string> = {};
  const generator = Object.create(pythonGenerator) as MockGenerator;
  generator.definitions_ = definitions;
  generator.quote_ = (value: string) => `'${value}'`;
  generator.provideFunction_ = (name: string, code: string) => {
    const resolved = code
      .split(generator.FUNCTION_NAME_PLACEHOLDER_)
      .join(name);
    definitions[`FN_${name}`] = resolved;
    return name;
  };
  generator.valueToCode = (_block, name, order) => {
    expect(order).toBe(Order.NONE);
    return values[name] ?? "";
  };
  return generator;
}

function modelFieldBlock(modelId: string): Block {
  return {
    getFieldValue: (field: string) => {
      if (field === "MODEL_ID") return modelId;
      throw new Error(`unexpected field ${field}`);
    },
  } as unknown as Block;
}

function joinedDefs(generator: MockGenerator): string {
  return Object.values(generator.definitions_).join("\n");
}

describe("start_model generator", () => {
  it("starts the selected model through the SDK", () => {
    const generator = createMockGenerator();
    const code = start_model(modelFieldBlock("hand_tracking"), generator);

    expect(code).toBe("start_model_with_sdk('hand_tracking')\n");
    const defs = joinedDefs(generator);
    expect(defs).toContain("from pib_sdk import Models");
    expect(defs).toContain(
      "with Models(host=rosbridge_host, port=9090) as models:",
    );
    expect(defs).toContain("models.start_model(str(model_id))");
    expect(defs).not.toContain("StartModel");
    expect(defs).not.toContain("rclpy");
  });
});

describe("stop_model generator", () => {
  it("stops the selected model through the SDK", () => {
    const generator = createMockGenerator();
    const code = stop_model(modelFieldBlock("face_detection"), generator);

    expect(code).toBe("stop_model_with_sdk('face_detection')\n");
    const defs = joinedDefs(generator);
    expect(defs).toContain("from pib_sdk import Models");
    expect(defs).toContain("models.stop_model(str(model_id))");
    expect(defs).not.toContain("StopModel");
    expect(defs).not.toContain("rclpy");
  });

  it("stops every model this program owns when All is selected", () => {
    const generator = createMockGenerator();
    const code = stop_model(modelFieldBlock(STOP_ALL_MODELS_VALUE), generator);

    expect(code).toBe("stop_all_models_with_sdk()\n");
    expect(code).not.toMatch(/stop_model\(/);
    expect(code).not.toMatch(/'[^']+'/);
    expect(code).not.toMatch(/"[^"]+"/);

    const defs = joinedDefs(generator);
    expect(defs).toContain("from pib_sdk import Models");
    expect(defs).toContain("models.stop_all_models()");
    expect(defs).not.toMatch(/stop_all_models\([^)]+\)/);
    expect(defs).not.toContain("models.stop_model(");
  });
});

function imitateHandsBlock(side: string, parts: string): Block {
  return {
    getFieldValue: (field: string) => {
      if (field === "SIDE") return side;
      if (field === "PARTS") return parts;
      throw new Error(`unexpected field ${field}`);
    },
  } as unknown as Block;
}

describe("imitate_hands generator", () => {
  it("emits a call with dropdown side and parts", () => {
    const generator = createMockGenerator();
    const code = imitate_hands(
      imitateHandsBlock("both", "all"),
      generator,
    );
    expect(code).toBe("imitate_hands('both', 'all')\n");
    const defs = joinedDefs(generator);
    expect(defs).toContain("hand_tracking_fast");
    expect(defs).toContain("apply_joint_trajectory_client");
    expect(defs).not.toContain("BusPriority");
    expect(defs).toContain("cdeg * math.pi / 18000.0");
  });

  it("executes generated helper with mock ROS and detection message", () => {
    const generator = createMockGenerator();
    imitate_hands(imitateHandsBlock("both", "all"), generator);
    const imitateDef = generator.definitions_.FN_imitate_hands;
    expect(imitateDef).toBeDefined();

    const harness = `
import math

class DetectionArray:
    pass

class ApplyJointTrajectory:
    class Request:
        pass

class JointTrajectory:
    pass

class JointTrajectoryPoint:
    pass

class Future:
    def __init__(self):
        self._done = True
    def done(self):
        return self._done

class Det:
    def __init__(self, handedness, keypoints):
        self.scalar_names = ["handedness"]
        self.scalar_values = [handedness]
        self.keypoint_x = [p[0] for p in keypoints]
        self.keypoint_y = [p[1] for p in keypoints]
        self.keypoint_z = [p[2] for p in keypoints]

class Msg:
    def __init__(self, detections):
        self.detections = detections
        self.frame_width = 1280.0
        self.frame_height = 720.0

calls = []
class Client:
    def call_async(self, request):
        jt = request.joint_trajectory
        calls.append(
            (list(jt.joint_names), [p * 18000.0 / math.pi for p in jt.points[0].positions])
        )
        return Future()

apply_joint_trajectory_client = Client()

class Node:
    def create_subscription(self, *args, **kwargs):
        return object()

node = Node()

spin_seq = {"count": 0}

def spin_once(node, timeout_sec=0.0):
    spin_seq["count"] += 1
    if spin_seq["count"] == 2:
        # Inject a person's right hand (mirrors to robot left)
        kps = [(0.0, 0.0, 0.0)] * 21
        kps[0] = (640.0, 360.0, 0.0)
        for i in range(1, 21):
            kps[i] = (640.0 + i * 5.0, 360.0 + i * 2.0, 0.0)
        # Bend index finger (5->6 along +X, 8->7 along +Z) so 3D angle is 90 deg (pi/2)
        kps[5] = (640.0, 360.0, 0.0)
        kps[6] = (768.0, 360.0, 0.0)
        kps[8] = (768.0, 360.0, 0.1)
        kps[7] = (768.0, 360.0, 0.0)
        msg = Msg([Det(0.9, kps)])
        _imitate_hands_state["msg"] = msg
        _imitate_hands_state["seq"] = 1

import logging
import sys
import time

class _Rclpy:
    spin_once = staticmethod(spin_once)

rclpy = _Rclpy()
sys.modules["rclpy"] = rclpy

${imitateDef}

_imitate_hands_state["last_cdeg"].clear()
_imitate_hands_state["last_seq"] = -1
_imitate_hands_state["seq"] = 0
_imitate_hands_state["msg"] = None
_imitate_hands_state["sub"] = None
_imitate_hands_state["future"] = None
_imitate_hands_state["no_hand_sent"] = False

imitate_hands("both", "all")
assert len(calls) == 1, calls
names, cdegs = calls[0]
by_name = dict(zip(names, cdegs))
assert "elbow_left" in by_name
# 90-deg index bend in isotropic 3D (x/1280, y/1280, z): -(pi/2 * 5000 - 9000) ~= 1146.0 cdeg
assert abs(by_name["index_left_stretch"] - (-(math.pi / 2.0 * 5000.0 - 9000.0))) < 1.0, by_name["index_left_stretch"]
assert all(-9000.0 <= v <= 9000.0 for v in cdegs)

# New frame with identical hand position -> deadband suppresses send
_imitate_hands_state["seq"] = 2
calls.clear()
imitate_hands("both", "all")
assert calls == [], "deadband should suppress unchanged joint targets on new frame"

# Empty hand frame -> rest pose uses positive shoulder_horizontal_left=9000.0 and shoulder_horizontal_right=7500.0
# and negative lower_arm_right_rotation=-1100.0
_imitate_hands_state["msg"] = Msg([])
_imitate_hands_state["seq"] = 3
imitate_hands("both", "all")
assert len(calls) == 1, calls
rest_by_name = dict(zip(calls[0][0], calls[0][1]))
assert rest_by_name["shoulder_horizontal_left"] == 9000.0, rest_by_name
assert rest_by_name["shoulder_horizontal_right"] == 7500.0, rest_by_name
assert rest_by_name["lower_arm_right_rotation"] == -1100.0, rest_by_name

# Person left hand (handedness < 0.5) mirrors to robot right arm with lower_arm_right_rotation=-7400.0
calls.clear()
kps_left = [(640.0 + i * 5.0, 360.0 + i * 2.0, 0.0) for i in range(21)]
_imitate_hands_state["msg"] = Msg([Det(0.1, kps_left)])
_imitate_hands_state["seq"] = 4
imitate_hands("both", "all")
assert len(calls) == 1, calls
right_by_name = dict(zip(calls[0][0], calls[0][1]))
assert right_by_name["lower_arm_right_rotation"] == -7400.0, right_by_name
`;

    execFileSync("python3", ["-c", harness], {
      encoding: "utf-8",
      stdio: ["pipe", "pipe", "pipe"],
    });
  });
});

describe("stop_model dropdown", () => {
  it("always offers All with a sentinel that cannot collide with a model id", () => {
    const stopOptions = getStopModelDropdownOptions();
    const startOptions = getModelDropdownOptions();

    expect(stopOptions[0]).toEqual(["All", STOP_ALL_MODELS_VALUE]);
    expect(STOP_ALL_MODELS_VALUE.startsWith("__")).toBe(true);
    expect(stopOptions.filter(([label]) => label === "All")).toHaveLength(1);
    expect(startOptions.some(([, value]) => value === STOP_ALL_MODELS_VALUE)).toBe(
      false,
    );
  });
});
