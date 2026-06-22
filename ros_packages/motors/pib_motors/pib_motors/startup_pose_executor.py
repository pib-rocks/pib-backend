import time
from typing import Any
from rclpy.node import Node
from pib_motors.motor import Motor
from datatypes.srv import ApplyJointTrajectory
from pib_api_client import motor_client, pose_client
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class StartupPoseExecutor:
    STARTUP_POSE_NAME: str = "Startup/Resting"
    STARTUP_POSE_VELOCITY: int = 1000  # Reduced velocity for startup pose
    DELAY_BETWEEN_MOTORS: float = (
        1.0  # Delay between moving each motor to allow for sequential execution
    )
    ALL_MOTORS_TIMEOUT: float = 15.0  # Limit: max wait for all motors combined
    WAIT_INTERVAL: float = 0.01

    def __init__(self, node: Node, motors: list[Motor]):
        self.node = node
        self.motors = motors

    def execute(self) -> bool:
        motor_positions = self._load_startup_pose()
        if motor_positions is None:
            self.node.get_logger().error("Startup pose execution aborted")
            return False

        original_settings = self._backup_motor_settings()
        self._reduce_motor_speeds(original_settings)

        try:
            success = self._move_sequentially(motor_positions)
        finally:
            self._restore_settings(original_settings)

        return success

    def _load_startup_pose(self) -> list[dict[str, Any]] | None:

        successful, pose = pose_client.get_pose_by_name(self.STARTUP_POSE_NAME)

        if not successful or pose is None:
            self.node.get_logger().warn(
                f"Could not find startup pose '{self.STARTUP_POSE_NAME}'"
            )
            return None

        successful, motor_positions = pose_client.get_motor_positions_of_pose(
            pose["poseId"]
        )
        if not successful:
            self.node.get_logger().warn(
                f"Could not load motor positions for '{self.STARTUP_POSE_NAME}'"
            )
            return None

        return motor_positions["motorPositions"]

    def _backup_motor_settings(self) -> dict[str, dict[str, Any]]:
        original_settings = {}
        for motor in self.motors:
            successful, settings = motor_client.get_motor_settings(motor.name)
            if successful:
                original_settings[motor.name] = settings.copy()
        return original_settings

    def _reduce_motor_speeds(
        self, original_settings: dict[str, dict[str, Any]]
    ) -> None:
        for motor in self.motors:
            if motor.name in original_settings:
                reduced_speed_settings = original_settings[motor.name].copy()
                reduced_speed_settings["velocity"] = self.STARTUP_POSE_VELOCITY
                # do not enable motors yet
                reduced_speed_settings["turnedOn"] = False

                motor.apply_settings(reduced_speed_settings)

    def _move_sequentially(self, motor_positions: list[dict[str, Any]]) -> bool:
        moved_motors: list[Motor] = []

        for motor_position in motor_positions:
            motor_name = motor_position["motorName"]
            position = motor_position["position"]

            motor = next((m for m in self.motors if m.name == motor_name), None)
            if motor is None:
                continue

            if not motor.check_if_motor_is_connected():
                self.node.get_logger().info(
                    f"Skipping motor '{motor_name}' (not connected)"
                )
                continue

            # 1. set position with reduced speed but motor turned off
            jt = JointTrajectory()
            jt.joint_names = [motor_name]
            point = JointTrajectoryPoint()
            point.positions.append(position)
            jt.points.append(point)

            req = ApplyJointTrajectory.Request()
            req.joint_trajectory = jt

            self.node.get_logger().info(
                f"Moving motor '{motor_name}' to startup position..."
            )
            self.node.apply_joint_trajectory(req, ApplyJointTrajectory.Response())

            # 2. then enable motor
            settings = motor.get_settings()
            settings["turnedOn"] = True
            motor.apply_settings(settings)
            self.node.get_logger().info(f"Enabling motor '{motor_name}'...")
            moved_motors.append(motor)

            # 3. short delay to stagger motor activations
            time.sleep(self.DELAY_BETWEEN_MOTORS)

        # 4. wait for ALL motors to finish before restoring speed
        self._wait_for_all_motors(moved_motors)

        return True

    def _wait_for_all_motors(self, motors: list[Motor]) -> None:
        start = time.time()
        while time.time() - start < self.ALL_MOTORS_TIMEOUT:
            if all(
                not m.check_if_motor_is_connected() or m.has_reached_position()
                for m in motors
            ):
                return
            time.sleep(self.WAIT_INTERVAL)
        self.node.get_logger().warn(
            "Not all motors reached startup position before restoring settings"
        )

    def _restore_settings(self, original_settings: dict[str, dict[str, Any]]) -> None:
        for motor in self.motors:
            if motor.name in original_settings:
                motor.apply_settings(original_settings[motor.name])
