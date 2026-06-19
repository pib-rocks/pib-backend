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
    STARTUP_TIMEOUT: float = (
        5.0  # Maximum time to wait for each motor to reach its startup position
    )
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

    # First move then enable:
    def _move_sequentially(self, motor_positions: list[dict[str, Any]]) -> bool:
        for motor_position in motor_positions:
            motor_name = motor_position["motorName"]
            position = motor_position["position"]

            motor = next((m for m in self.motors if m.name == motor_name), None)
            if motor is None:
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

            if motor.check_if_motor_is_connected():
                if not self._wait_for_motor(motor):
                    self.node.get_logger().warn(
                        f"Motor '{motor_name}' did not reach startup position in time"
                    )

        return True

    def _wait_for_motor(self, motor: Motor, timeout: float | None = None) -> bool:
        if timeout is None:
            timeout = self.STARTUP_TIMEOUT
        start = time.time()

        while time.time() - start < timeout:
            if motor.has_reached_position():
                return True
            time.sleep(self.WAIT_INTERVAL)

        return False

    def _restore_settings(self, original_settings: dict[str, dict[str, Any]]) -> None:
        for motor in self.motors:
            if motor.name in original_settings:
                motor.apply_settings(original_settings[motor.name])
