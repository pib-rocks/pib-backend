# PR-1657: Blockly — use pib-sdk get_hand_position_xyz for relative move (A) + new 'Get hand position' block (B)

Ticket: https://pib-rocks.atlassian.net/browse/PR-1657
Repo: pib-backend — /home/pib/opencode/pib-backend, branch `PR-1657` (from develop `754c0382`).
User instruction: implement; ON SUCCESS merge to develop.

## Goal
- (A) Refactor `set_hand_position_xyz` RELATIVE mode to reuse `pib-sdk.get_hand_position_xyz(side)` instead of the inline `get_current_hand_position` body. Absolute mode unchanged.
- (B) Add a new standalone Blockly block "Get hand position" (SIDE input left/right → X/Y/Z number outputs in mm) whose generator calls the same SDK method. Generate correct code, add discriminating tests.

## Verified facts
- set_hand_position_xyz block def: `pib_blockly/pib_blockly_server/src/pib-blockly/program-blocks/motor-blocks.ts` (lines ~67-117): MODE dropdown (ABSOLUTE/RELATIVE), SIDE dropdown (left/right), X/Y/Z input_value. colour 355.
- set_hand_position_xyz generator: `pib_blockly/pib_blockly_server/src/pib-blockly/program-generators/motor-generators.ts` (fn lines 106-160). RELATIVE mode currently emits GET_CURRENT_HAND_POSITION_FUNCTION.
- Inline helper: `pib_blockly/pib_blockly_server/src/pib-blockly/program-generators/util/function-declarations.ts` GET_CURRENT_HAND_POSITION_FUNCTION (lines 58-72): reads get_joint_position(motor)/100.0 per arm motor, calls fk(side, q).translation. This is what (A) should replace with get_hand_position_xyz(side).
- SET_HAND_POSITION_XYZ_FUNCTION (lines 74-96): RELATIVE uses current; ABSOLUTE uses x/y/z directly; then ik(side, xyz=target) + Write(...).move(arm, *q_deg). Keep this shape.
- Tests: `tests/blockly_generator/motor_generator.test.ts` — mock generator with definitions_, provideFunction_, valueToCode; createHandPositionBlock(side, mode) pattern (lines 40-51). Run with `npx jest` (jest.config.js in tests/blockly_generator, cwd the blockly_generator dir).
- pib-sdk method (since PR-1655): get_hand_position_xyz(side, *, telemetry_host="localhost", telemetry_port=9090, telemetry=None) -> (x,y,z) mm. Reads arm.motor_names via Telemetry.get_positions_deg, runs fk. Exported from pib_sdk.

## IMPORTANT runtime note — pib-sdk version
The installed pib_sdk here is version 0.4 and does NOT have get_hand_position_xyz yet (verified hasattr=False). Method is merged on main but not published to PyPI (next release v0.5.0). The ros-programs runtime installs `pib-sdk` unpinned from PyPI.
ACTION: pin the program-runtime requirement to include the method: `ros_packages/requirements.txt` line 11 is `pib-sdk` → set `pib-sdk>=0.5.0` (guards against missing method). Document that live execution only works once v0.5.0 is published. The generator/tests themselves only check emitted code strings (no installed method needed).

## Work items

### A. Refactor relative mode
In motor-generators.ts set_hand_position_xyz (RELATIVE branch):
- Replace emission of IMPORT_GET_JOINT_POSITION / INIT_GET_JOINT_POSITION_CLIENT / GET_JOINT_POSITION_FUNCTION / GET_CURRENT_HAND_POSITION_FUNCTION with a single import of get_hand_position_xyz from pib_sdk.
- Generated set_hand_position_xyz runtime fn calls get_hand_position_xyz(side) for current, target = [current[0]+x, current[1]+y, current[2]+z for RELATIVE; ABSOLUTE unchanged.
- Remove now-unused inline helper GET_CURRENT_HAN_POSITION_FUNCTION (and its imports) from function-declarations.ts if unused.

### B. New 'Get hand position' block
1. Block def in motor-blocks.ts: type `get_hand_position_xyz`, message "Get hand position of %1", SIDE field_dropdown (left/right). Expose X/Y/Z. Blockly multiple-value-output is nonstandard; RECOMENDED: a block whose `output` connection yields the `(x,y,z)` triple, OR three small output blocks (`get_hand_x`, `_y`, `_z`) each calling the header and indexing. CHOOSE ONE, implement consistently with tests. colour 355.
2. Generator: emit `from pib_dk import get_hand_position_xyz` and a call returning x/y/z (or index into triple).
3. tooltip/helpUrl document the "last commanded position" caveat.

### C. Tests (motor_generator.test.ts or new file)
- (A) RELATIVE case asserts emitted code contains get_hand_position_xyz(side) and no longer the inline get_current_hand_position body.
- (B) New block cases: emitted code contains get_hand_position_xyz( and correct return/index for x,y,z.
- Keep existing tests green.

## Hard constraints
1. Stay on branch PR-1657. On SUCCESS merge to develop (user-authorized); do not push prematurely.
2. Only touch blockly server + generator + block defs + tests + pib-sdk requirement line. No changes to pib_sdk source, motor logic, routes.
3. Clean readable TS following existing patterns.
4. Do NOT commit .hermes/plans/**. You MAY commit (`feat: PR-1657 ...` / `refactor:` for A).
5. Report REAL results.

## Verify (orchestrator)
- `cd tests/blockly_generator && npx jest 2>&1 | tail` → all pass incl new cases.
- git diff --stat shows motor-blocks.ts, motor-generators.ts, function-declarations.ts (removals), test file, ros_packages/requirements.txt.
- Inline GET_CURRENT_HAND_POSITION_FUNCTION gone/unused after (A).
- ABSOLUTE mode logic unchanged.
- git status clean except .hermes.