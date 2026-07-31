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

ACTUATING_TOOLS = {
    "pib_move_motor",
    "pib_apply_pose",
    "pib_run_program",
    "pib_set_led",
    "pib_set_relay",
}
MAX_SOUL_LESSON_CHARS = 500

NonEmptyString = Annotated[str, Field(min_length=1, strict=True)]
MotorPosition = Annotated[int, Field(strict=True)]
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
            "PIB_MCP_ENABLE_ACTUATION=true in the server environment."
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
    def pib_list_motors() -> dict[str, Any]:
        """List configured motors and bricklets, including live motor positions."""

        return _invoke(robot.list_motors)

    @server.tool(annotations=read_annotations, structured_output=True)
    def pib_get_state() -> dict[str, Any]:
        """Return current joint state, diagnostics, and robot telemetry."""

        return _invoke(robot.get_state)

    @server.tool(annotations=read_annotations, structured_output=True)
    def pib_list_poses() -> dict[str, Any]:
        """List stored robot poses."""

        return _invoke(robot.list_poses)

    @server.tool(annotations=read_annotations, structured_output=True)
    def pib_list_programs() -> dict[str, Any]:
        """List stored Blockly/Python programs."""

        return _invoke(robot.list_programs)

    @server.tool(annotations=read_annotations, structured_output=True)
    def pib_capture_image() -> dict[str, Any]:
        """Capture one camera frame as a base64-encoded JPEG."""

        return _invoke(robot.capture_image)

    @server.tool(annotations=actuation_annotations, structured_output=True)
    def pib_move_motor(
        motor_name: NonEmptyString, position: MotorPosition
    ) -> dict[str, Any]:
        """Move one motor after validating its configured rotation limits."""

        disabled = require_actuation("pib_move_motor")
        if disabled:
            return disabled
        motor_name = motor_name.strip()
        if not motor_name:
            return _error("invalid_parameter", "motor_name must not be blank")
        try:
            motors = robot.get_motors()
        except BackendError as exc:
            return _error(exc.code, exc.message, exc.details)
        motor = next(
            (item for item in motors if item.get("name") == motor_name), None
        )
        if motor is None:
            return _error("motor_not_found", f"unknown motor {motor_name!r}")
        minimum = motor.get("rotationRangeMin", motor.get("rotation_range_min"))
        maximum = motor.get("rotationRangeMax", motor.get("rotation_range_max"))
        if not isinstance(minimum, (int, float)) or not isinstance(
            maximum, (int, float)
        ):
            return _error(
                "motor_limits_unavailable",
                f"configured limits for motor {motor_name!r} are unavailable",
            )
        if position < minimum or position > maximum:
            return _error(
                "position_out_of_range",
                (
                    f"position {position} is outside motor {motor_name!r} "
                    f"limits [{minimum}, {maximum}]"
                ),
                {"minimum": minimum, "maximum": maximum, "requested": position},
            )
        return _invoke(robot.move_motor, motor_name, position)

    @server.tool(annotations=actuation_annotations, structured_output=True)
    def pib_apply_pose(pose_name: NonEmptyString) -> dict[str, Any]:
        """Apply a stored pose by its exact name."""

        disabled = require_actuation("pib_apply_pose")
        if disabled:
            return disabled
        pose_name = pose_name.strip()
        if not pose_name:
            return _error("invalid_parameter", "pose_name must not be blank")
        return _invoke(robot.apply_pose, pose_name)

    @server.tool(annotations=actuation_annotations, structured_output=True)
    def pib_run_program(program_id: NonEmptyString) -> dict[str, Any]:
        """Start a stored program by program number/ID."""

        disabled = require_actuation("pib_run_program")
        if disabled:
            return disabled
        program_id = program_id.strip()
        if not program_id:
            return _error("invalid_parameter", "program_id must not be blank")
        return _invoke(robot.run_program, program_id)

    @server.tool(annotations=actuation_annotations, structured_output=True)
    def pib_set_led(
        button_id: ButtonId,
        red: ColorChannel,
        green: ColorChannel,
        blue: ColorChannel,
    ) -> dict[str, Any]:
        """Set one RGB button LED (button 1-3, channels 0-255)."""

        disabled = require_actuation("pib_set_led")
        if disabled:
            return disabled
        return _invoke(robot.set_led, button_id, red, green, blue)

    @server.tool(annotations=actuation_annotations, structured_output=True)
    def pib_set_relay(turned_on: bool) -> dict[str, Any]:
        """Turn the solid-state relay on or off."""

        disabled = require_actuation("pib_set_relay")
        if disabled:
            return disabled
        return _invoke(robot.set_relay, turned_on)

    @server.tool(annotations=write_annotations, structured_output=True)
    def pib_soul_append(
        personality_id: NonEmptyString, lesson: SoulLesson
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
        return _invoke(robot.soul_append, personality_id, lesson)

    return server


mcp = create_server()


def main() -> None:
    """Run the pib MCP server over stdio."""

    mcp.run(transport="stdio")
