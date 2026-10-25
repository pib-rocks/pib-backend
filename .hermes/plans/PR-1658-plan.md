# PR-1658: Blockly pose save (C1) + pose value/utility blocks (B/D) from pib-sdk

Ticket: https://pib-rocks.atlassian.net/browse/PR-1658
Repo: pib-backend — /home/pib/opencode/pib-backend, branch `PR-1658` (from develop `43e2cac4`).
Instruction: implement; ON SUCCESS create a GitHub pull-request PR-1658 → develop (do not direct-push to develop).

## Scope
- C1: 'Save current pose as' statement block — captures ALL motor positions including the head (via
  pib-sdk All / _all_motor_names expansion), no range/arm selector, single NAME input. Generated code
  calls pib_sdk save_current_pose with the full motor set.
- B/D: 'get_all_poses' (value list), 'get_pose_joints' (value motor->angle deg), 'has_pose' (bool),
  'pose_count' (number).

## Verified facts (pib-backend, this repo)
- Block defs: `pib_blockly/pib_blockly_server/src/pib-blockly/program-blocks/pose-block.ts` (existing
  `move_to_pose`, Blockly.Blocks["move_to_pose"], init with CustomFieldDropdown + getPoses() placeholder).
- Block registry: `pib_blockly/pib_blockly_server/src/pib-blockly/program-blocks/custom-blocks.ts`
  calls `Blockly.common.defineBlocks(moveToPose)`; register new blocks here too.
- Generator: `pib_blockly/pib_blockly_server/src/pib-blockly/program-generators/pose-generator.ts`.
  Existing moveToPoseGenerator uses definitions (IMPORT_PIB_SDK, IMPORT_POSE_CLIENT, INIT_ROS...) +
  provideFunction_("apply_pose", APPLY_POSE_FUNCTION). Note: existing move_to_pose uses
  `pib_api_client.pose_client` + `pib_sdk.Write` (APPLY_POSE_FUNCTION), NOT pib_sdk.features.poses.
- util/definitions.ts has IMPORT_* constants (IMPORT_PIB_SDK, IMPORT_OS, IMPORT_LOGGING, CONFIGURE_LOGGING, INIT_ROS, IMPORT_RCLPY, IMPORT_SYS, IMPORT_POSE_CLIENT).
- Tests: `tests/blockly_generator/pose_generator.test.ts` — mock generator (definitions_, provideFunction_, no valueToCode). Existing case asserts `apply_pose("<uuid>")\n`.
- Run: `cd tests/blockly_generator && node node_modules/jest/bin/jest.js --config jest.config.js` (npx jest may be non-executable).

## Verified pib-sdk API (from the installed source / repo)
- `pib_sdk.features.poses.save_current_pose(telemetry, backend, name, motor_names) -> Pose`: reads
  telemetry.get_positions_deg(motor_names), calls backend.create_pose(name, positions). Requires a
  Telemetry(host) instance and a BackendClient(host).
- `pib_sdk.features.poses.list_poses(backend) -> list[PoseSummary(pose_id,name,deletable)]`.
- `pib_sdk.features.poses.get_pose(backend, *, name=None, pose_id=None) -> Pose(..., motor_angles_deg={motor:deg})`.
- `pib_sdk.control.All` token expands (pib_sdk/control.py _all_motor_names) to the union of every motor
  group including RIGHT_ARM+LEFT_ARM+HEAD (e.g. turn_head_motor, tilt_forward_motor). This is the intended
  "all motors incl. head" source for C1.

## C1 design decision (important)
Existing move_to_pose uses `pib_api_client.pose_client` + `pib_sdk.Write` (REST via pib_backend API client),
NOT pib_sdk.features.poses. For consistency you MUST decide ONE runtime path. Because the ticket requires
pib-sdk All expansion and save_current_pose (which itself needs Telemetry + BackendClient), the generated
code for C1 and B/D SHOULD use `pib_sdk.features.poses` with a Telemetry + BackendClient created from the
ROSBRIDGE_HOST + backend host (matching how other generators resolve rosbridge host, e.g.
`os.getenv("ROSBRIDGE_HOST", "rosbridge-ws")`). Keep move_to_pose's existing path untouched unless you can
unify cleanly (do NOT break it).

## Work items

### A. Block definitions (pose-block.ts)
Add to pose-block.ts (or a new pose-blocks file registered the same way):
- `save_current_pose`: statement block, message "Save current pose as %1" with a text/string input NAME
  (field_input NAME). NO arm/range dropdown. Tooltip: captures ALL motors incl. head; saved position is the
  last COMMANDED position (not live encoder).
- `get_all_poses`: value block, output (list) — generated list_poses(backend).
- `get_pose_joints`: value block with a pose NAME input/output (map) — generated get_pose(...).motor_angles_deg.
- `has_pose`: boolean block with a NAME input — true/false if pose exists.
- `pose_count`: number block — len(list_poses(backend)).
Register all in custom-blocks.ts.

### B. Generators (pose-generator.ts or new file; register generator mapping like existing custom-generators)
Emit the imports (from pib_sdk import ...) and calls. Concretely:
- C1: emit `from pib_sdk.features.poses import save_current_pose` (correct SDK path; note existing
  move_to_pose uses pib_api_client.pose_client, keep that path intact), open a Telemetry(host) +
  BackendClient(host), expand All -> motor_names, call save_current_pose(...). Provide the needed imports
  (IMPORT_* + a new one for All/save_current_pose) in generator.definitions_.
- B1/D2: `list_poses(backend)` usage; D1: existence check; B2: get_pose(...).motor_angles_deg.
Use the same provideFunction_/definitions_ pattern as moveToPoseGenerator.

### C. Tests (pose_generator.test.ts or a new pose-blocks test file)
- C1: emitted code contains save_current_pose with the full motor set including head (assert the emitted
  motor_names / All expansion includes turn_head_motor / tilt_forward_motor), NO arm-range dropdown.
- B1/D2: get_all_poses / pose_count emit list_poses.
- D1: has_pose emits an existence check.
- B2: get_pose_joints emits get_pose(...).motor_angles_deg.
- Keep existing pose_generator tests green.

## Hard constraints
1. Stay on branch `PR-1658`. Do NOT direct-push to develop (merge happens via GitHub PR).
2. Do NOT modify pib-sdk source, motor-control logic, routes, or the existing move_to_pose behavior.
3. Do NOT add an arm/range selector to save_current_pose — full motor set incl. head by default.
4. Do NOT commit `.hermes/plans/**`. You MAY commit on PR-1658 (`feat: PR-1658 ...` / `refactor:`).
5. Report REAL results; do not fabricate.

## Verify (orchestrator)
- `cd tests/blockly_generator && node node_modules/jest/bin/jest.js --config jest.config.js` → all pass incl new cases.
- `git diff --stat develop...PR-1658` shows pose-block.ts (or new block file), custom-blocks.ts, pose-generator.ts (or new gen file), util/definitions.ts (new imports), the test file.
- grep confirms save_current_pose uses All/motor_names incl head; no range dropdown in the block def.
- existing move_to_pose test still green.