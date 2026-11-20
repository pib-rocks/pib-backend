import * as Blockly from "blockly";
import { Block } from "blockly/core/block";
import { Order, pythonGenerator } from "blockly/python";
import {
    get_all_poses,
    get_pose_joints,
    has_pose,
    moveToPoseGenerator,
    play_pose_sequence,
    pose_count,
    save_current_pose,
    save_detection_as_pose,
} from "../../pib_blockly/pib_blockly_server/src/pib-blockly/program-generators/pose-generator";
import {
    PlayPoseSequenceBlock,
    moveToPose,
    poseBlocks,
} from "../../pib_blockly/pib_blockly_server/src/pib-blockly/program-blocks/pose-block";

type MockGenerator = typeof pythonGenerator & {
  definitions_: Record<string, string>;
  provideFunction_: (name: string, code: string) => string;
  valueToCode: (
    block: Block,
    name: string,
    order: Order,
  ) => string;
};

function createMockGenerator(): MockGenerator {
  const definitions: Record<string, string> = {};
  const generator = Object.create(pythonGenerator) as MockGenerator;
  generator.definitions_ = definitions;
  generator.provideFunction_ = (name: string, code: string) => {
    // Real Blockly substitutes its function-name placeholder before storing the
    // definition; the mock has to do the same or an assertion on "def <name>"
    // can never match.
    const resolved = code
      .split(generator.FUNCTION_NAME_PLACEHOLDER_)
      .join(name);
    definitions[`FN_${name}`] = resolved;
    return name;
  };
  return generator;
}

describe("moveToPoseGenerator", () => {
  it("skips a pose that no longer exists instead of failing the program", () => {
    // A stored program keeps the pose UUID it was built with; re-recording a pose
    // allocates a new UUID, so the old reference answers 404. Both generated helpers must
    // skip it with one line instead of ending the program in a traceback (measured on
    // .168: BackendError GET /pose/by-name/<uuid> -> HTTP 404 and no motion).
    const generator = createMockGenerator();
    moveToPoseGenerator(
      { getFieldValue: () => "gone-pose" } as unknown as Block,
      generator,
    );
    play_pose_sequence(
      {
        itemCount_: 1,
        getFieldValue: (name: string) => (name === "POSE_0" ? "gone-pose" : 1),
      } as unknown as Block,
      generator,
    );

    const defs = Object.values(generator.definitions_).join("\n");
    expect(defs).toContain('logging.warning(f"pose {poseId} is not available');
    expect(defs).toContain('print(f"pose {identifier} is not available');
    expect(defs).toContain("continue");
  });

  it("generates apply_pose call with pose UUID and drives the pose in centi-degrees (no +-90 cap)", () => {
    const poseId = "aaaaaaaa-bbbb-cccc-dddd-eeeeeeeeeeee";
    const block = {
      getFieldValue: (field: string) => {
        if (field === "POSE") return poseId;
        throw new Error(`unexpected field ${field}`);
      },
    } as unknown as Block;

    const generator = createMockGenerator();
    const code = moveToPoseGenerator(block, generator);

    expect(code).toBe(`apply_pose("${poseId}")\n`);
    const defs = Object.values(generator.definitions_).join("\n");
    expect(defs).toContain("from pib_api_client import pose_client");
    expect(defs).toContain("import pib_sdk");
    expect(defs).toContain("centidegrees.append(float(motor_position[\"position\"]))");
    expect(defs).toContain("with pib_sdk.Write(host=rosbridge_host, port=9090) as writer:");
    expect(defs).toContain("writer.send_timed_trajectory(names, [(centidegrees, 1.0)])");
    // A finger servo legitimately travels past +-90 degrees; move() would refuse it.
    expect(defs).not.toContain("writer.move(*args)");
  });

    it("registers move_to_pose without polluting Blockly.Blocks with init or getPoses", () => {
        delete Blockly.Blocks["init"];
        delete Blockly.Blocks["getPoses"];
        Blockly.common.defineBlocks(moveToPose);

        expect(Blockly.Blocks["move_to_pose"]).toBeDefined();
        expect(typeof Blockly.Blocks["move_to_pose"].getPoses).toBe("function");
        expect(Blockly.Blocks["init"]).toBeUndefined();
        expect(Blockly.Blocks["getPoses"]).toBeUndefined();

        const workspace = new Blockly.Workspace();
        Blockly.Events.disable();
        try {
            const block = workspace.newBlock("move_to_pose");
            Blockly.Blocks["move_to_pose"].getPoses = () => [
                ["Head up", "pose-1"],
            ];
            const field = block.getField("POSE") as Blockly.FieldDropdown;
            expect(field.getOptions(false)).toEqual([["Head up", "pose-1"]]);
        } finally {
            workspace.dispose();
            Blockly.Events.enable();
        }
    });
});

function blockWithName(name: string): Block {
  return {
    getFieldValue: (field: string) => {
      if (field === "NAME") return name;
      throw new Error(`unexpected field ${field}`);
    },
  } as unknown as Block;
}

function generatorWithNameValue(nameCode?: string): MockGenerator {
  const generator = createMockGenerator();
  generator.valueToCode = (block, name, order) => {
    expect(name).toBe("NAME");
    expect(order).toBe(Order.NONE);
    return nameCode ?? "";
  };
  return generator;
}

describe("pose SDK generators", () => {
  it("saves every motor through All and explicitly includes the head", () => {
    const generator = generatorWithNameValue("'rest \"pose\"'");
    const code = save_current_pose({} as Block, generator);

    expect(code).toBe(
      "save_current_pose_with_all_motors('rest \"pose\"')\n",
    );
    const defs = Object.values(generator.definitions_).join("\n");
    expect(defs).toContain(
      "from pib_sdk.features.poses import save_current_pose, list_poses, get_pose",
    );
    expect(defs).toContain(
      "from pib_sdk.control import All, _expand_motor_specs",
    );
    expect(defs).toContain("motor_names = _expand_motor_specs([All])");
    expect(defs).toContain('"turn_head_motor" in motor_names');
    expect(defs).toContain('"tilt_forward_motor" in motor_names');
    expect(defs).toContain(
      "save_current_pose(telemetry, pose_backend, name, motor_names)",
    );
  });

  it("generates the all-poses list expression", () => {
    const generator = createMockGenerator();
    expect(get_all_poses({} as Block, generator)).toEqual([
      "list_poses(pose_backend)",
      Order.FUNCTION_CALL,
    ]);
  });

  it("generates the named pose joints map expression", () => {
    const generator = createMockGenerator();
    expect(get_pose_joints(blockWithName("wave"), generator)).toEqual([
      "get_pose(pose_backend, name='wave').motor_angles_deg",
      Order.MEMBER,
    ]);
  });

  it("generates a boolean pose existence expression", () => {
    const generator = createMockGenerator();
    expect(has_pose(blockWithName("wave"), generator)).toEqual([
      "any(pose.name == 'wave' for pose in list_poses(pose_backend))",
      Order.FUNCTION_CALL,
    ]);
  });

  it("generates the pose count expression", () => {
    const generator = createMockGenerator();
    expect(pose_count({} as Block, generator)).toEqual([
      "len(list_poses(pose_backend))",
      Order.FUNCTION_CALL,
    ]);
  });

  it("initializes the SDK backend from the configured Flask API URL", () => {
    const generator = createMockGenerator();
    get_all_poses({} as Block, generator);
    const defs = Object.values(generator.definitions_).join("\n");

    expect(defs).toContain("from pib_sdk.backend import BackendClient");
    expect(defs).toContain(
      'os.getenv("FLASK_API_BASE_URL", "http://flask-app:5000")',
    );
    expect(defs).toContain("pose_backend = BackendClient(");
  });
});

describe("play_pose_sequence generator", () => {
  beforeAll(() => {
    Blockly.common.defineBlocks(moveToPose);
    Blockly.common.defineBlocks(poseBlocks);
  });

  function createSequenceGenerator(sequenceCode?: string): MockGenerator {
    const generator = createMockGenerator();
    generator.valueToCode = (block, name, order) => {
      expect(name).toBe("SEQUENCE");
      expect(order).toBe(Order.NONE);
      return sequenceCode ?? "";
    };
    return generator;
  }

  it("emits a timed helper call from dynamic pose rows and prepends t=0.0 initial joint angles", () => {
    Blockly.Blocks["move_to_pose"].getPoses = () => [
      ["Wave 1", "pose-uuid-1"],
      ["Wave 2", "pose-uuid-2"],
      ["Rest", "pose-uuid-3"],
    ];
    const workspace = new Blockly.Workspace();
    Blockly.Events.disable();
    try {
      const block = workspace.newBlock(
        "play_pose_sequence",
      ) as PlayPoseSequenceBlock;
      block.setFieldValue("pose-uuid-1", "POSE_0");
      block.setFieldValue(1.2, "DURATION_0");
      block.setFieldValue("pose-uuid-2", "POSE_1");
      block.setFieldValue(2.0, "DURATION_1");

      const generator = createMockGenerator();
      const code = play_pose_sequence(block, generator);

      expect(code).toBe(
        '_pib_play_pose_sequence_timed([("pose-uuid-1", 1.2), ("pose-uuid-2", 2)])\n',
      );
      const defs = Object.values(generator.definitions_).join("\n");
      expect(defs).toContain("from pib_sdk.telemetry import Telemetry");
      expect(defs).toContain("import time");
      expect(defs).toContain("pose = get_pose(pose_backend, pose_id=identifier)");
      expect(defs).toContain("pose = get_pose(pose_backend, name=identifier)");
      expect(defs).toContain("live_joints = telemetry.get_positions_deg(joint_names)");
      expect(defs).toContain("writer.send_timed_trajectory(joint_names, waypoints)");
      expect(defs).toContain("import sys");
      expect(defs).toContain("import logging");
      expect(defs).toContain("live_initial_loaded");
      expect(defs).toContain("play_pose_sequence: reached waypoint");
    } finally {
      workspace.dispose();
      Blockly.Events.enable();
    }
  });

  it("keeps the first waypoint when live initial positions fail and logs waypoint progress (PR-2178)", () => {
    const { execFileSync } = require("child_process");
    const generator = createSequenceGenerator("");
    play_pose_sequence({} as Block, generator);
    const defs = Object.values(generator.definitions_).join("\n");

    const script = `
import logging, os, sys, time, types

records = []
class _Capture(logging.Handler):
    def emit(self, rec):
        records.append((rec.levelname, rec.getMessage()))

logging.getLogger().setLevel(logging.INFO)
logging.getLogger().addHandler(_Capture())

class _Pose:
    def __init__(self, name, angles):
        self.name = name
        self.pose_id = name
        self.motor_angles_deg = angles

poses = {
    "pose_a": _Pose("pose_a", {"turn_head_motor": 10.0}),
    "pose_b": _Pose("pose_b", {"turn_head_motor": 25.0}),
}
def _get_pose(_backend, pose_id=None, name=None):
    key = pose_id or name
    if key in poses:
        return poses[key]
    raise RuntimeError("missing")

sent = []

class _FailingTelemetry:
    def __init__(self, host="", port=9090):
        raise TimeoutError("rosbridge busy")

class _Writer:
    def __enter__(self): return self
    def __exit__(self, *a): return False
    def send_timed_trajectory(self, names, waypoints):
        sent.append((list(names), list(waypoints)))

# Register stub pib_sdk package so generated imports execute cleanly without ROS.
mod_sdk = types.ModuleType("pib_sdk")
mod_sdk.Write = lambda host="", port=9090: _Writer()
mod_backend = types.ModuleType("pib_sdk.backend")
mod_backend.BackendClient = lambda host="", port=5000: object()
mod_features = types.ModuleType("pib_sdk.features")
mod_poses = types.ModuleType("pib_sdk.features.poses")
mod_poses.save_current_pose = lambda *a, **kw: None
mod_poses.list_poses = lambda *a, **kw: []
mod_poses.get_pose = _get_pose
mod_poses.play_pose_sequence_timed = lambda *a, **kw: None
mod_telemetry = types.ModuleType("pib_sdk.telemetry")
mod_telemetry.Telemetry = _FailingTelemetry

sys.modules["pib_sdk"] = mod_sdk
sys.modules["pib_sdk.backend"] = mod_backend
sys.modules["pib_sdk.features"] = mod_features
sys.modules["pib_sdk.features.poses"] = mod_poses
sys.modules["pib_sdk.telemetry"] = mod_telemetry

${defs}

logging.getLogger().setLevel(logging.INFO)
logging.getLogger().addHandler(_Capture())
time.sleep = lambda _s: None
_pib_play_pose_sequence_timed([("pose_a", 0.5), ("pose_b", 0.5)])
assert len(sent) == 1, sent
names, wps = sent[0]
assert names == ["turn_head_motor"], names
# Must NOT skip pose_a: t=0.0 at pose_a, t=0.5 at pose_a (held/driven), t=1.0 at pose_b
assert len(wps) == 3, wps
assert wps[1][0] == [1000.0] and wps[1][1] == 0.5, wps
assert wps[2][0] == [2500.0] and wps[2][1] == 1.0, wps
msgs = [m for _, m in records]
assert any("live initial joint positions unavailable" in m for m in msgs), msgs
assert any("reached waypoint 1/2 (pose_a)" in m for m in msgs), msgs
assert any("reached waypoint 2/2 (pose_b)" in m for m in msgs), msgs

# Also verify live-initial path: when robot is ALREADY at pose_a (10.0 deg), waypoint 1 starts at t=0.00s
records.clear()
sent.clear()
class _LiveTelemetryAtPoseA:
    def __init__(self, host="", port=9090):
        self.ros = None
    def __enter__(self): return self
    def __exit__(self, *a): return False
    def get_positions_deg(self, _names):
        return {"turn_head_motor": 10.0}

Telemetry = _LiveTelemetryAtPoseA
_pib_play_pose_sequence_timed([("pose_a", 0.5), ("pose_b", 0.5)])
assert len(sent) == 1, sent
names2, wps2 = sent[0]
assert len(wps2) == 2, wps2
assert wps2[0][0] == [1000.0] and wps2[0][1] == 0.0, wps2
assert wps2[1][0] == [2500.0] and wps2[1][1] == 0.5, wps2
msgs2 = [m for _, m in records]
assert any("already within 0 cdeg of live position" in m for m in msgs2), msgs2
assert any("reached waypoint 2/2 (pose_b)" in m for m in msgs2), msgs2
print("OK")
`;
    const out = execFileSync("python3", ["-c", script], { encoding: "utf-8" });
    expect(out.trim().split("\n").pop()).toBe("OK");
    expect(out).toContain("reached waypoint 1/2 (pose_a)");
    expect(out).toContain("reached waypoint 2/2 (pose_b)");
  });

  it("supports legacy SEQUENCE value input fallback when dynamic rows are absent", () => {
    const sequenceLiteral = "[['wave', 1.5], ['rest', 2]]";
    const generator = createSequenceGenerator(sequenceLiteral);
    const code = play_pose_sequence({} as Block, generator);

    expect(code).toBe(`_pib_play_pose_sequence_timed(${sequenceLiteral})\n`);
  });

  it("defaults a missing sequence input to an empty list", () => {
    const generator = createSequenceGenerator("");
    expect(play_pose_sequence({} as Block, generator)).toBe(
      "_pib_play_pose_sequence_timed([])\n",
    );
  });
});

describe("save_current_pose generator", () => {
  it("emits the helper call with a connected string literal", () => {
    const generator = generatorWithNameValue('"wave"');
    expect(save_current_pose({} as Block, generator)).toBe(
      'save_current_pose_with_all_motors("wave")\n',
    );
  });

  it("passes a connected variable through instead of a hardcoded name", () => {
    const generator = generatorWithNameValue("pose_name");
    expect(save_current_pose({} as Block, generator)).toBe(
      "save_current_pose_with_all_motors(pose_name)\n",
    );
  });

  it("falls back to a default name when the input is unconnected", () => {
    const generator = generatorWithNameValue("");
    expect(save_current_pose({} as Block, generator)).toBe(
      'save_current_pose_with_all_motors("pose name")\n',
    );
  });
});

describe("save_detection_as_pose generator", () => {
  it("reuses the current-pose helper with a connected string literal", () => {
    const generator = generatorWithNameValue('"detected_grip"');
    const code = save_detection_as_pose({} as Block, generator);

    expect(code).toBe(
      'save_current_pose_with_all_motors("detected_grip")\n',
    );
    const defs = Object.values(generator.definitions_).join("\n");
    expect(defs).toContain(
      "from pib_sdk.features.poses import save_current_pose, list_poses, get_pose",
    );
    expect(defs).toContain(
      "save_current_pose(telemetry, pose_backend, name, motor_names)",
    );
  });

  it("passes a connected name variable through instead of a hardcoded pose name", () => {
    const generator = generatorWithNameValue("pose_from_detection");
    expect(save_detection_as_pose({} as Block, generator)).toBe(
      "save_current_pose_with_all_motors(pose_from_detection)\n",
    );
  });

  it("falls back to the default pose name when the input is unconnected", () => {
    const generator = generatorWithNameValue("");
    expect(save_detection_as_pose({} as Block, generator)).toBe(
      'save_current_pose_with_all_motors("pose name")\n',
    );
  });
});

describe("save current pose block", () => {
  beforeAll(() => Blockly.common.defineBlocks(poseBlocks));

  it("has a string value input for the name and no arm or range dropdown", () => {
    const workspace = new Blockly.Workspace();
    Blockly.Events.disable();
    try {
      const block = workspace.newBlock("save_current_pose");
      const nameInput = block.getInput("NAME");

      expect(nameInput).not.toBeNull();
      expect(nameInput?.type).toBe(Blockly.inputs.inputTypes.VALUE);
      expect(nameInput?.connection?.getCheck()).toEqual(["String"]);
      expect(block.getField("NAME")).toBeNull();
      expect(block.previousConnection).not.toBeNull();
      expect(block.nextConnection).not.toBeNull();
      expect(block.getField("SIDE")).toBeNull();
      expect(block.getField("RANGE")).toBeNull();
      expect(
        block.inputList.flatMap((input) => input.fieldRow).some(
          (field) => field instanceof Blockly.FieldDropdown,
        ),
      ).toBe(false);
    } finally {
      workspace.dispose();
      Blockly.Events.enable();
    }
  });

  it("defines save_detection_as_pose as a statement with a string NAME input", () => {
    const workspace = new Blockly.Workspace();
    Blockly.Events.disable();
    try {
      const block = workspace.newBlock("save_detection_as_pose");
      const nameInput = block.getInput("NAME");

      expect(nameInput).not.toBeNull();
      expect(nameInput?.type).toBe(Blockly.inputs.inputTypes.VALUE);
      expect(nameInput?.connection?.getCheck()).toEqual(["String"]);
      expect(block.previousConnection).not.toBeNull();
      expect(block.nextConnection).not.toBeNull();
      expect(block.outputConnection).toBeNull();
    } finally {
      workspace.dispose();
      Blockly.Events.enable();
    }
  });

  it("defines play_pose_sequence as a dynamic statement block supporting arbitrary middle insertion, deletion, and serialization", () => {
    Blockly.common.defineBlocks(moveToPose);
    Blockly.Blocks["move_to_pose"].getPoses = () => [
      ["Pose A", "id-a"],
      ["Pose B", "id-b"],
      ["Pose C", "id-c"],
      ["Pose D", "id-d"],
    ];
    const workspace = new Blockly.Workspace();
    Blockly.Events.disable();
    try {
      const block = workspace.newBlock(
        "play_pose_sequence",
      ) as PlayPoseSequenceBlock;

      expect(block.previousConnection).not.toBeNull();
      expect(block.nextConnection).not.toBeNull();
      expect(block.outputConnection).toBeNull();
      expect(block.getPoseSequenceCount()).toBe(2);

      block.setFieldValue("id-a", "POSE_0");
      block.setFieldValue(1.0, "DURATION_0");
      block.setFieldValue("id-c", "POSE_1");
      block.setFieldValue(2.5, "DURATION_1");

      // Insert an intermediate pose between Pose A (index 0) and Pose C (index 1)
      block.insertStepAt(1, "id-b", 1.75);
      expect(block.getPoseSequenceCount()).toBe(3);
      expect(block.getStepsState()).toEqual([
        { pose: "id-a", duration: 1.0 },
        { pose: "id-b", duration: 1.75 },
        { pose: "id-c", duration: 2.5 },
      ]);

      // Insert at the beginning (index 0)
      block.insertStepAt(0, "id-d", 0.8);
      expect(block.getPoseSequenceCount()).toBe(4);
      expect(block.getStepsState().map((s) => s.pose)).toEqual([
        "id-d",
        "id-a",
        "id-b",
        "id-c",
      ]);

      // Remove from the middle (index 2 -> "id-b")
      block.removeStepAt(2);
      expect(block.getPoseSequenceCount()).toBe(3);
      expect(block.getStepsState()).toEqual([
        { pose: "id-d", duration: 0.8 },
        { pose: "id-a", duration: 1.0 },
        { pose: "id-c", duration: 2.5 },
      ]);

      // Verify saveExtraState / loadExtraState round-trip
      const saved = block.saveExtraState();
      const restored = workspace.newBlock(
        "play_pose_sequence",
      ) as PlayPoseSequenceBlock;
      restored.loadExtraState(saved);
      expect(restored.getPoseSequenceCount()).toBe(3);
      expect(restored.getStepsState()).toEqual([
        { pose: "id-d", duration: 0.8 },
        { pose: "id-a", duration: 1.0 },
        { pose: "id-c", duration: 2.5 },
      ]);

      // Verify gear mutator decompose/compose reordering
      const mutatorWorkspace = new Blockly.Workspace();
      const container = restored.decompose(mutatorWorkspace);
      restored.compose(container);
      expect(restored.getPoseSequenceCount()).toBe(3);
      expect(restored.getStepsState()).toEqual([
        { pose: "id-d", duration: 0.8 },
        { pose: "id-a", duration: 1.0 },
        { pose: "id-c", duration: 2.5 },
      ]);
      mutatorWorkspace.dispose();
    } finally {
      workspace.dispose();
      Blockly.Events.enable();
    }
  });
});
