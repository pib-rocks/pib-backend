"""FastMCP server exposing pib robot capabilities as validated tools."""

from __future__ import annotations

import os
from typing import Annotated, Any, Callable

try:
    from mcp.server.fastmcp import FastMCP
except ImportError:
    try:
        from mcp.server.mcpserver import FastMCP
    except ImportError:
        from mcp.server.mcpserver import MCPServer as FastMCP
from mcp.types import ToolAnnotations
from pydantic import Field

from .backend import BackendError, PibBackend
from .tool_budgets import DEFAULT_TOOL_PAYLOAD_BUDGET_BYTES, TOOL_PAYLOAD_BUDGET_BYTES

# UTF-8 JSON byte budgets for default (compact) tool results; see tool_budgets.py.
TOOL_PAYLOAD_BUDGETS = {
    **TOOL_PAYLOAD_BUDGET_BYTES,
    "default": DEFAULT_TOOL_PAYLOAD_BUDGET_BYTES,
}

ACTUATING_TOOLS = {
    "move_motor",
    "move_motors",
    "apply_joint_trajectory",
    "apply_pose",
    "run_program",
    "set_led",
    "set_relay",
    "control_smarthome_device",
    "trigger_smarthome_scene",
}
MAX_SOUL_LESSON_CHARS = 500

NonEmptyString = Annotated[str, Field(min_length=1, strict=True)]
MotorPositionDegrees = Annotated[float, Field(strict=True)]
ButtonId = Annotated[int, Field(ge=1, le=3, strict=True)]
ColorChannel = Annotated[int, Field(ge=0, le=255, strict=True)]
SoulLesson = Annotated[
    str, Field(min_length=1, max_length=MAX_SOUL_LESSON_CHARS, strict=True)
]


def _env_enabled(name: str, default: bool = False) -> bool:
    value = os.getenv(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _success(result: Any) -> dict[str, Any]:
    return {"ok": True, "result": result}


def _error(code: str, message: str, details: Any = None) -> dict[str, Any]:
    error = {"code": code, "message": message}
    if details is not None:
        error["details"] = details
    return {"ok": False, "error": error}


def _parse_smarthome_value(value: str) -> Any:
    text = (value or "").strip()
    if not text:
        return None
    try:
        if "." in text:
            return float(text)
        return int(text)
    except ValueError:
        return text


def _invoke(operation: Callable[..., Any], *args: Any) -> dict[str, Any]:
    try:
        return _success(operation(*args))
    except BackendError as exc:
        return _error(exc.code, exc.message, exc.details)
    except Exception as exc:
        return _error("backend_error", str(exc))


def create_server(
    backend: Any | None = None, actuating_enabled: bool | None = None
) -> FastMCP:
    """Create an independently testable server with an injectable backend."""

    robot = backend or PibBackend()
    can_actuate = (
        _env_enabled("PIB_MCP_ENABLE_ACTUATION")
        if actuating_enabled is None
        else actuating_enabled
    )
    server = FastMCP(
        "pib",
        instructions=(
            "Schema-validated pib robot tools. Actuator tools require "
            "PIB_MCP_ENABLE_ACTUATION=true in the server environment. "
            "A gesture or a head turn while an answer is spoken is "
            "apply_pose, move_motor, or move_motors, not a separate gesture system. "
            "Motor angles at the tool boundary are in degrees; ROS trajectories use radians."
        ),
    )

    read_annotations = ToolAnnotations(
        readOnlyHint=True, destructiveHint=False, idempotentHint=True
    )
    actuation_annotations = ToolAnnotations(
        readOnlyHint=False, destructiveHint=True, idempotentHint=False
    )
    write_annotations = ToolAnnotations(
        readOnlyHint=False, destructiveHint=False, idempotentHint=False
    )

    def require_actuation(tool_name: str) -> dict[str, Any] | None:
        if can_actuate:
            return None
        return _error(
            "actuation_disabled",
            (
                f"{tool_name} is disabled; set "
                "PIB_MCP_ENABLE_ACTUATION=true when starting the server"
            ),
        )

    @server.tool(annotations=read_annotations, structured_output=True)
    def list_motors(detail: bool = False) -> dict[str, Any]:
        """List motors with live positions; pass detail=true for full config and bricklets."""

        return _invoke(robot.list_motors, detail)

    @server.tool(annotations=read_annotations, structured_output=True)
    def get_state(detail: bool = False) -> dict[str, Any]:
        """Return joint state and diagnostics; pass detail=true for full motor config."""

        return _invoke(robot.get_state, detail)

    @server.tool(annotations=read_annotations, structured_output=True)
    def get_motion_model(detail: bool = False) -> dict[str, Any]:
        """Return the live motion model from the robot database.

        Default is the rendered prompt and units; pass detail=true for kinematic
        chains, joint limits, and named postures.
        """
        return _invoke(robot.get_motion_model, detail)

    @server.tool(annotations=read_annotations, structured_output=True)
    def list_poses(detail: bool = False) -> dict[str, Any]:
        """List stored robot poses; pass detail=true for motor positions."""

        return _invoke(robot.list_poses, detail)

    @server.tool(annotations=read_annotations, structured_output=True)
    def list_programs(detail: bool = False) -> dict[str, Any]:
        """List stored Blockly/Python programs; pass detail=true for Blockly code."""

        return _invoke(robot.list_programs, detail)

    @server.tool(annotations=read_annotations, structured_output=True)
    def capture_image(detail: bool = False) -> dict[str, Any]:
        """Capture one camera frame; default is a reference and metadata only."""

        return _invoke(robot.capture_image, detail)

    def _motor_limits_degrees(
        motor: dict[str, Any],
    ) -> tuple[float, float] | None:
        minimum = motor.get("rotationRangeMin", motor.get("rotation_range_min"))
        maximum = motor.get("rotationRangeMax", motor.get("rotation_range_max"))
        if not isinstance(minimum, (int, float)) or not isinstance(
            maximum, (int, float)
        ):
            return None
        return minimum / 100.0, maximum / 100.0

    def _validate_motor_moves(
        moves: list[dict[str, Any]],
    ) -> dict[str, Any] | None:
        if not moves:
            return _error("invalid_parameter", "moves must not be empty")
        try:
            motors = robot.get_motors()
        except BackendError as exc:
            return _error(exc.code, exc.message, exc.details)
        motors_by_name = {item.get("name"): item for item in motors if item.get("name")}
        validated: list[dict[str, Any]] = []
        for move in moves:
            motor_name = str(move.get("motor", "")).strip()
            if not motor_name:
                return _error(
                    "invalid_parameter", "each move needs a non-blank motor name"
                )
            position_degrees = move.get("position_degrees")
            if not isinstance(position_degrees, (int, float)):
                return _error(
                    "invalid_parameter",
                    f"position_degrees for motor {motor_name!r} must be a number",
                )
            position_degrees = float(position_degrees)
            motor = motors_by_name.get(motor_name)
            if motor is None:
                return _error("motor_not_found", f"unknown motor {motor_name!r}")
            limits = _motor_limits_degrees(motor)
            if limits is None:
                return _error(
                    "motor_limits_unavailable",
                    f"configured limits for motor {motor_name!r} are unavailable",
                )
            minimum, maximum = limits
            if position_degrees < minimum or position_degrees > maximum:
                return _error(
                    "position_out_of_range",
                    (
                        f"position {position_degrees}° is outside motor {motor_name!r} "
                        f"limits [{minimum}, {maximum}]°"
                    ),
                    {
                        "minimum": minimum,
                        "maximum": maximum,
                        "requested": position_degrees,
                    },
                )
            validated.append(
                {"motor": motor_name, "position_degrees": position_degrees}
            )
        return (
            None
            if validated
            else _error("invalid_parameter", "moves must not be empty")
        )

    @server.tool(annotations=actuation_annotations, structured_output=True)
    def move_motors(moves: list[dict[str, Any]]) -> dict[str, Any]:
        """Move several motors in one trajectory (positions in degrees).

        Each entry needs ``motor`` and ``position_degrees``. All entries are
        validated before any motion is requested; one invalid entry rejects the
        whole call.
        """

        disabled = require_actuation("move_motors")
        if disabled:
            return disabled
        validation_error = _validate_motor_moves(moves)
        if validation_error:
            return validation_error
        validated_moves = [
            {
                "motor": str(move.get("motor", "")).strip(),
                "position_degrees": float(move["position_degrees"]),
            }
            for move in moves
        ]
        return _invoke(robot.move_motors, validated_moves)

    @server.tool(annotations=actuation_annotations, structured_output=True)
    def move_motor(
        motor_name: NonEmptyString, position_degrees: MotorPositionDegrees
    ) -> dict[str, Any]:
        """Move one motor in degrees after validating its configured rotation limits."""

        disabled = require_actuation("move_motor")
        if disabled:
            return disabled
        motor_name = motor_name.strip()
        if not motor_name:
            return _error("invalid_parameter", "motor_name must not be blank")
        validation_error = _validate_motor_moves(
            [{"motor": motor_name, "position_degrees": position_degrees}]
        )
        if validation_error:
            return validation_error
        return _invoke(robot.move_motor, motor_name, float(position_degrees))

    @server.tool(annotations=actuation_annotations, structured_output=True)
    def apply_joint_trajectory(
        joint_names: list[NonEmptyString],
        points: list[dict[str, Any]],
    ) -> dict[str, Any]:
        """Apply a timed multi-waypoint joint trajectory (tool positions in degrees).

        ``points`` is a list of waypoints. Each waypoint provides
        ``positions_degrees`` with one angle per ``joint_names`` entry (same order)
        and optional ``time_from_start_sec`` for timed motion. Values are converted
        to radians on the ROS wire.
        """

        disabled = require_actuation("apply_joint_trajectory")
        if disabled:
            return disabled
        names = [name.strip() for name in joint_names if name and name.strip()]
        if not names:
            return _error("invalid_parameter", "joint_names must not be empty")
        if len(names) != len(set(names)):
            return _error("invalid_parameter", "joint_names must be unique")
        if not points:
            return _error("invalid_parameter", "points must not be empty")
        normalized_points: list[dict[str, Any]] = []
        for index, point in enumerate(points):
            positions = point.get("positions_degrees")
            if not isinstance(positions, list) or not positions:
                return _error(
                    "invalid_parameter",
                    f"point {index} must include positions_degrees with one value per joint",
                )
            if len(positions) != len(names):
                return _error(
                    "invalid_parameter",
                    (
                        f"point {index} has {len(positions)} positions for "
                        f"{len(names)} joint names"
                    ),
                )
            try:
                degrees = [float(value) for value in positions]
            except (TypeError, ValueError):
                return _error(
                    "invalid_parameter",
                    f"point {index} positions_degrees must be numeric",
                )
            waypoint: dict[str, Any] = {"positions_degrees": degrees}
            if (
                "time_from_start_sec" in point
                and point["time_from_start_sec"] is not None
            ):
                try:
                    waypoint["time_from_start_sec"] = float(
                        point["time_from_start_sec"]
                    )
                except (TypeError, ValueError):
                    return _error(
                        "invalid_parameter",
                        f"point {index} time_from_start_sec must be numeric",
                    )
            normalized_points.append(waypoint)
        return _invoke(robot.apply_joint_trajectory, names, normalized_points)

    @server.tool(annotations=actuation_annotations, structured_output=True)
    def apply_pose(pose_name: NonEmptyString) -> dict[str, Any]:
        """Apply a stored pose by its exact name."""

        disabled = require_actuation("apply_pose")
        if disabled:
            return disabled
        pose_name = pose_name.strip()
        if not pose_name:
            return _error("invalid_parameter", "pose_name must not be blank")
        return _invoke(robot.apply_pose, pose_name)

    @server.tool(annotations=actuation_annotations, structured_output=True)
    def run_program(program_id: NonEmptyString) -> dict[str, Any]:
        """Start a stored program by program number/ID."""

        disabled = require_actuation("run_program")
        if disabled:
            return disabled
        program_id = program_id.strip()
        if not program_id:
            return _error("invalid_parameter", "program_id must not be blank")
        return _invoke(robot.run_program, program_id)

    @server.tool(annotations=actuation_annotations, structured_output=True)
    def set_led(
        button_id: ButtonId,
        red: ColorChannel,
        green: ColorChannel,
        blue: ColorChannel,
    ) -> dict[str, Any]:
        """Set one RGB button LED (button 1-3, channels 0-255)."""

        disabled = require_actuation("set_led")
        if disabled:
            return disabled
        return _invoke(robot.set_led, button_id, red, green, blue)

    @server.tool(annotations=actuation_annotations, structured_output=True)
    def set_relay(turned_on: bool) -> dict[str, Any]:
        """Turn the solid-state relay on or off."""

        disabled = require_actuation("set_relay")
        if disabled:
            return disabled
        return _invoke(robot.set_relay, turned_on)

    @server.tool(annotations=read_annotations, structured_output=True)
    def list_smarthome_devices(
        room: str = "", domain: str = "", detail: bool = False
    ) -> dict[str, Any]:
        """List voice-exposed SmartHome devices, optionally filtered by room or domain."""

        return _invoke(
            robot.list_smarthome_devices,
            room.strip() or None,
            domain.strip() or None,
            detail,
        )

    @server.tool(annotations=read_annotations, structured_output=True)
    def get_smarthome_state(
        device: NonEmptyString, detail: bool = False
    ) -> dict[str, Any]:
        """Read the current state of one voice-exposed SmartHome device."""

        return _invoke(robot.get_smarthome_state, device.strip(), detail)

    @server.tool(annotations=actuation_annotations, structured_output=True)
    def control_smarthome_device(
        device: NonEmptyString,
        action: NonEmptyString,
        value: str = "",
    ) -> dict[str, Any]:
        """Run a SmartHome action on one voice-exposed device."""

        disabled = require_actuation("control_smarthome_device")
        if disabled:
            return disabled
        return _invoke(
            robot.control_smarthome_device,
            device.strip(),
            action.strip(),
            _parse_smarthome_value(value),
        )

    @server.tool(annotations=actuation_annotations, structured_output=True)
    def trigger_smarthome_scene(scene: NonEmptyString) -> dict[str, Any]:
        """Activate a SmartHome scene by name."""

        disabled = require_actuation("trigger_smarthome_scene")
        if disabled:
            return disabled
        return _invoke(robot.trigger_smarthome_scene, scene.strip())

    @server.tool(annotations=write_annotations, structured_output=True)
    def soul_append(
        personality_id: NonEmptyString,
        lesson: SoulLesson,
        detail: bool = False,
    ) -> dict[str, Any]:
        """Append one durable lesson to a personality SOUL; never replace it."""

        personality_id = personality_id.strip()
        lesson = lesson.strip()
        if not personality_id:
            return _error("invalid_parameter", "personality_id must not be blank")
        if not lesson:
            return _error("invalid_parameter", "lesson must not be blank")
        if len(lesson) > MAX_SOUL_LESSON_CHARS:
            return _error(
                "lesson_too_long",
                f"lesson exceeds the {MAX_SOUL_LESSON_CHARS}-character limit",
                {
                    "maximum": MAX_SOUL_LESSON_CHARS,
                    "actual": len(lesson),
                },
            )
        return _invoke(robot.soul_append, personality_id, lesson, detail)

    return server


mcp = create_server()


def main() -> None:
    """Run the pib MCP server over stdio."""

    mcp.run(transport="stdio")
