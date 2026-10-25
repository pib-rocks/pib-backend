# PR-1663: Blockly — read a motor's current (Strom) into a variable via a motor dropdown

Ticket: https://pib-rocks.atlassian.net/browse/PR-1663
Repo: pib-backend — /home/pib/opencode/pib-backend, branch `PR-1663` (from develop `ccea2b71`).
Instruction: implement; ON SUCCESS create a pull-request PR-1663 -> develop (do not merge).

## Goal
Add a standalone value Blockly block that reads a motor's electrical current (milliamps) and can be
assigned to a variable. Motor selectable via a dropdown. Backed by pib-sdk `Telemetry.get_current_ma`.

## Verified facts (this repo / pib-sdk)
- Motor dropdown pattern: `program-blocks/motor-blocks.ts` `move_motor` has `field_dropdown MOTORNAME` with
  options like `["left shoulder vertical","SHOULDER_VERTICAL_LEFT"]`, `["elbow right","ELBOW_RIGHT"]`, ...
  (all finger + limbs + head). Reuse this SAME option list for the new motor-current block.
- `program-generators/motor-generators.ts` `motorOptionToMotorName` Map converts the dropdown token to the
  pib motor-name string (e.g. `SHOULDER_VERTICAL_LEFT` -> `shoulder_vertical_left`). Reuse this map
  (or a shared copy) for the new generator.
- Value-block generator pattern (from pose-generator.ts PR-1658): functions return `[code, Order]`,
  add imports to `generator.definitions_`, and are registered automatically because the module is spread
  into `custom-generators.ts` `const generators = { ...moveToPose, ... }` -> `pythonGenerator.forBlock[name]`.
- `definitions.ts` has `IMPORT_PIB_SDK_TELEMETRY = "from pib_sdk.telemetry import Telemetry"` (ready to use).
- rosbridge host pattern in generated code: `rosbridge_host = os.getenv("ROSBRIDGE_HOST", "rosbridge-ws")`;
  classes opened as `with Telemetry(host=rosbridge_host, port=9090) as t:` with `except: localhost`.

## Design
Add:
1. Block def in `program-blocks/motor-blocks.ts` (or the pose/current area): type `motor_current`,
   message `Motor current of %1` with a `field_dropdown MOTORNAME` using the same motor option list,
   `output: "Number"`, colour similar (355), tooltip: returns the motor's electrical current draw in
   milliamps (from the servo bricklet; may read 0/timeout for a motor without a connected bricklet).
   Register via the existing `createBlockDefinitionsFromJsonArray` array (or a new export included in it).
2. Generator in `program-generators/motor-generators.ts` (or a new current-generator module spread into
   custom-generators): 
   - read `MOTORNAME` -> `motorOptionToMotorName` -> motor_name.
   - add to `generator.definitions_`: `IMPORT_PIB_SDK_TELEMETRY` (+ `IMPORT_OS` for env, `IMPORT_LOGGING`)
     and define a helper `def...` that opens `Telemetry(host=rosbridge_host, port=9090)`, calls
     `get_current_ma(motor_name)`, returns the int mA, with localhost fallback on exception.
   - return `[helper_call(motor_name), Order.FUNCTION_CALL]`.
3. Tests in `tests/blockly_generator/` (add to motor_generator.test.ts or a new current test file): assert
   the emitted code contains `Telemetry.get_current_ma` (or the helper calling it) with the correct
   motor_name, and that it uses the dropdown->motor-name mapping.

## Hard constraints
1. Stay on branch `PR-1663`. Do NOT merge. Do NOT push. You MAY commit (`feat: PR-1663 ...`).
2. Reuse the existing motor dropdown list + motorOptionToMotorName mapping (do NOT create a divergent list).
3. Do NOT change existing move_motor / pose behavior. Do NOT modify pib_sdk source.
4. Do NOT commit `.hermes/plans/**`. Do NOT commit if tests fail.
5. Report REAL results.

## Verify (orchestrator)
- `cd tests/blockly_generator && node node_modules/jest/bin/jest.js --config jest.config.js` -> all pass incl new case.
- `git diff --name-only origin/develop...HEAD` shows the block file, generator, test.
- grep confirms generated code uses `get_current_ma` and the motor-name mapping.