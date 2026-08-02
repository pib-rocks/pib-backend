# Test Basis: Blockly Motor Control pib-sdk Refactoring

## Overview
This test basis specifies the refactoring of motor control code generators (`move_motor` and `moveToPoseGenerator`) in `pib_blockly` to replace ROS2 `rclpy` `ApplyJointTrajectory` service client boilerplate with high-level `pib_sdk.Write().move(...)` calls.

## Functional Requirements
- **pib-sdk Integration:** Generated Python functions for motor movement (`apply_joint_trajectory`) and pose application (`apply_pose`) must call `pib_sdk.Write().move(motor_name, position)` instead of manually creating `ApplyJointTrajectory` requests, creating ROS2 clients, and calling `spin_until_future_complete`.
- **Absolute Mode:** Generating code for `move_motor` in ABSOLUTE mode passes the target position directly to `apply_joint_trajectory(selected_motor, position)`.
- **Relative Mode:** Generating code for `move_motor` in RELATIVE mode gets the current motor position via `get_joint_position` and passes the calculated target position to `apply_joint_trajectory`.
- **Pose Generation:** Generating code for `moveToPoseGenerator` fetches the motor positions for the pose ID via `pose_client` and applies each motor position via `pib_sdk.Write().move(motor_name, position)`.
- **Module Definitions:** Code generation must include `import pib_sdk` in `generator.definitions_` and eliminate `ApplyJointTrajectory` ROS2 service imports and client initializations.

## Scenarios (BDD/Gherkin)

### Scenario 1: Move single motor in ABSOLUTE mode with pib-sdk
```gherkin
Given a user configures the "Move Motor" block with motor "ELBOW_LEFT", mode "ABSOLUTE", and position "1000"
When the Blockly Python code is generated
Then the generated Python code calls 'apply_joint_trajectory("elbow_left", 1000)'
And the function definition of 'apply_joint_trajectory' uses 'pib_sdk.Write().move(motor_name, position)'
And the definitions include 'import pib_sdk'
```

### Scenario 2: Move single motor in RELATIVE mode with pib-sdk
```gherkin
Given a user configures the "Move Motor" block with motor "TURN_HEAD", mode "RELATIVE", and position "250"
When the Blockly Python code is generated
Then the generated Python code calls 'apply_joint_trajectory("turn_head_motor", get_joint_position(\'turn_head_motor\') + 250)'
And the function definition of 'apply_joint_trajectory' uses 'pib_sdk.Write().move(motor_name, position)'
And the definitions include 'import pib_sdk' and 'INIT_GET_JOINT_POSITION_CLIENT'
```

### Scenario 3: Move to pose using pib-sdk
```gherkin
Given a user configures the "Move to Pose" block with pose ID "aaaaaaaa-bbbb-cccc-dddd-eeeeeeeeeeee"
When the Blockly Python code is generated
Then the generated Python code calls 'apply_pose("aaaaaaaa-bbbb-cccc-dddd-eeeeeeeeeeee")'
And the function definition of 'apply_pose' uses 'pib_sdk.Write().move(motor_name, position)'
And the definitions include 'import pib_sdk' and 'from pib_api_client import pose_client'
```

### Scenario 4: Motor name mapping verification
```gherkin
Given a user configures the "Move Motor" block with motor "TILT_FORWARD_HEAD", mode "ABSOLUTE", and position "0"
When the Blockly Python code is generated
Then the generated motor name string is "tilt_forward_motor"
And the generated Python code calls 'apply_joint_trajectory("tilt_forward_motor", 0)'
```
