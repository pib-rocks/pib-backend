# Test Basis: Global pib-sdk Installation & Motor Sweep Control

**Repository:** `pib-backend`  
**Jira Story:** `PR-1496`  
**Components:** `pib-sdk`, `ros_packages/programs`, `pib_blockly`, `rosbridge-ws`

---

## Overview

This test basis defines the requirements and behavior-driven development (BDD) scenarios for the global availability of `pib-sdk` across host and container execution environments, as well as joint motor sweep control via ROS 2 bridge services.

`pib-sdk` provides Python high-level APIs (`Write`, `head`, `right_arm`, `left_arm`, `left_hand`, `right_hand`, kinematics) to interact with pib robot motors over `rosbridge_websocket` (default port 9090) using `datatypes/ApplyJointTrajectory` and `datatypes/ApplyMotorSettings` services.

---

## Component Matrix

| Component | Interface / Environment | Dependency / Endpoint | Responsibility |
|---|---|---|---|
| System Host Python | Python 3.11/3.12 Host | Global `pib-sdk` package | CLI / local script execution using SDK |
| `ros-programs` Container | Docker Container | `ros_packages/programs/Dockerfile` | Execution runtime for generated Blockly / user Python programs |
| `pib_blockly` Server/Client | Node.js + Python | `pib_blockly_client` & `ros-programs` | Blockly block compilation to Python calling `pib-sdk` APIs |
| ROS Bridge | WebSocket (Port 9090) | `/apply_joint_trajectory`, `/apply_motor_settings` | Service bridging `pib-sdk` commands to ROS 2 nodes |

---

## BDD Specifications

### Scenario 1: Global Importability of `pib-sdk`
```gherkin
Given a Python host environment or container runtime (ros-programs / blockly)
When `import pib_sdk` is executed in Python
Then the module imports successfully without raising `ModuleNotFoundError`
And `pib_sdk.__version__` is accessible
And high-level tokens (`head`, `right_arm`, `left_arm`, `zero_position`) and control client `Write` are exposed
```

### Scenario 2: SDK Initialization and Rosbridge Connection
```gherkin
Given a running rosbridge websocket service on host "localhost" and port 9090
When `pib_sdk.Write(host="localhost", port=9090)` is initialized
Then a roslibpy client connection to the ROS bridge is established
And services `/apply_joint_trajectory` and `/apply_motor_settings` are registered
And closing the client terminates the ROS bridge connection cleanly
```

### Scenario 3: Motor Sweep Execution Between Two Angles
```gherkin
Given an initialized `pib_sdk.Write` client connected to rosbridge
When a motor sweep is executed for joint "head_pitch" from angle -15.0 degrees to 15.0 degrees
Then `Write.move("head_pitch", -15.0)` sends an `ApplyJointTrajectory` service request with converted position -1500.0
And `Write.move("head_pitch", 15.0)` sends an `ApplyJointTrajectory` service request with converted position 1500.0
And both service calls return `successful: true`
```

### Scenario 4: Motor Group Sweep Execution
```gherkin
Given an initialized `pib_sdk.Write` client connected to rosbridge
When `Write.move(head, -20.0)` is invoked followed by `Write.move(head, 20.0)`
Then the trajectory request expands the `head` group token to all constituent joints
And commands are issued across the entire group for both positions
```

### Scenario 5: Container Execution Environment Parity
```gherkin
Given the `ros_packages/programs/Dockerfile` and `ros_packages/requirements.txt` build configuration
When the container image is built
Then `pib-sdk` is installed into the Python runtime environment
And programs executing inside the `ros-programs` container can import and run `pib-sdk` without missing dependencies
```
