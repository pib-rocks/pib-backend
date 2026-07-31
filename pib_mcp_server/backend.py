"""Adapters from MCP tools to pib's HTTP API and ROS 2 services."""

from __future__ import annotations

import json
import os
import time
import uuid
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.parse import quote
from urllib.request import Request, urlopen


class BackendError(RuntimeError):
    """An operational failure that can be safely returned to an MCP client."""

    def __init__(self, code: str, message: str, details: Any = None):
        super().__init__(message)
        self.code = code
        self.message = message
        self.details = details


class PibBackend:
    """Use pib's existing REST and rosbridge interfaces.

    REST owns configuration and persisted data. ROS owns live hardware and
    program execution. Imports for websocket-client are deliberately lazy so
    read-only REST tools can still report useful errors in reduced environments.
    """

    def __init__(
        self,
        api_base_url: str | None = None,
        rosbridge_url: str | None = None,
        timeout: float | None = None,
    ):
        self.api_base_url = (
            api_base_url
            or os.getenv("PIB_MCP_API_BASE_URL")
            or os.getenv("FLASK_API_BASE_URL")
            or "http://localhost:5000"
        ).rstrip("/")
        self.rosbridge_url = (
            rosbridge_url
            or os.getenv("PIB_MCP_ROSBRIDGE_URL")
            or "ws://localhost:9090"
        )
        self.timeout = float(
            timeout
            if timeout is not None
            else os.getenv("PIB_MCP_REQUEST_TIMEOUT", "10")
        )

    def _http(
        self, method: str, path: str, payload: dict[str, Any] | None = None
    ) -> Any:
        data = None if payload is None else json.dumps(payload).encode("utf-8")
        request = Request(
            self.api_base_url + path,
            method=method,
            data=data,
            headers={"Content-Type": "application/json"} if data else {},
        )
        try:
            with urlopen(request, timeout=self.timeout) as response:
                body = response.read().decode("utf-8")
                return json.loads(body) if body else {}
        except HTTPError as exc:
            body = exc.read().decode("utf-8", errors="replace")[:500]
            raise BackendError(
                "api_error",
                f"pib API returned HTTP {exc.code}",
                {"status": exc.code, "body": body},
            ) from exc
        except (URLError, TimeoutError, ValueError) as exc:
            raise BackendError("api_unavailable", f"pib API request failed: {exc}") from exc

    def _ros_service(
        self, service: str, service_type: str, arguments: dict[str, Any]
    ) -> dict[str, Any]:
        try:
            import websocket
        except ImportError as exc:
            raise BackendError(
                "rosbridge_client_missing",
                "websocket-client is required for ROS hardware tools",
            ) from exc

        request_id = f"pib-mcp-{uuid.uuid4()}"
        request = {
            "op": "call_service",
            "id": request_id,
            "service": service,
            "type": service_type,
            "args": arguments,
        }
        connection = None
        try:
            connection = websocket.create_connection(
                self.rosbridge_url, timeout=self.timeout
            )
            connection.send(json.dumps(request))
            deadline = time.monotonic() + self.timeout
            while time.monotonic() < deadline:
                response = json.loads(connection.recv())
                if response.get("id") != request_id:
                    continue
                if response.get("result") is False:
                    raise BackendError(
                        "ros_service_failed",
                        response.get("message") or f"ROS service {service} failed",
                    )
                values = response.get("values")
                if isinstance(values, dict):
                    return values
            raise BackendError(
                "ros_timeout", f"ROS service {service} timed out after {self.timeout:g}s"
            )
        except BackendError:
            raise
        except Exception as exc:
            raise BackendError(
                "ros_unavailable", f"ROS service {service} could not be called: {exc}"
            ) from exc
        finally:
            if connection is not None:
                connection.close()

    def get_motors(self) -> list[dict[str, Any]]:
        payload = self._http("GET", "/motor")
        return list(payload.get("motors", []))

    def list_motors(self) -> dict[str, Any]:
        motors = self.get_motors()
        for motor in motors:
            name = motor.get("name")
            if not name:
                continue
            try:
                position = self._ros_service(
                    "/get_joint_position",
                    "datatypes/srv/GetJointPosition",
                    {"joint_name": name},
                )
                motor["currentPosition"] = (
                    position.get("position") if position.get("successful") else None
                )
                if not position.get("successful"):
                    motor["positionError"] = position.get("message", "unavailable")
            except BackendError as exc:
                motor["currentPosition"] = None
                motor["positionError"] = exc.message
        bricklets = self._http("GET", "/bricklet").get("bricklets", [])
        return {"motors": motors, "bricklets": bricklets}

    def get_state(self) -> dict[str, Any]:
        state = self.list_motors()
        state["diagnostics"] = self._http("GET", "/diagnostics/summary")
        return state

    def list_poses(self) -> list[dict[str, Any]]:
        return list(self._http("GET", "/pose").get("poses", []))

    def list_programs(self) -> list[dict[str, Any]]:
        return list(self._http("GET", "/program").get("programs", []))

    def capture_image(self) -> dict[str, Any]:
        response = self._ros_service(
            "/get_camera_image", "datatypes/srv/GetCameraImage", {}
        )
        image = response.get("image_base64", response.get("imageBase64", ""))
        if not image:
            raise BackendError("camera_empty", "camera returned no image")
        return {"imageBase64": image, "mimeType": "image/jpeg"}

    def move_motor(self, motor_name: str, position: int) -> dict[str, Any]:
        response = self._ros_service(
            "/apply_joint_trajectory",
            "datatypes/srv/ApplyJointTrajectory",
            {
                "joint_trajectory": {
                    "joint_names": [motor_name],
                    "points": [{"positions": [position]}],
                }
            },
        )
        if not response.get("successful"):
            raise BackendError("motion_failed", f"motor {motor_name} did not move")
        return {"motorName": motor_name, "position": position}

    def apply_pose(self, pose_name: str) -> dict[str, Any]:
        pose = self._http("GET", f"/pose/by-name/{quote(pose_name, safe='')}")
        motor_positions = pose.get("motorPositions", pose.get("motor_positions", []))
        if not motor_positions:
            raise BackendError("pose_empty", f"pose {pose_name!r} has no motor positions")
        names = [
            item.get("motorName", item.get("motor_name")) for item in motor_positions
        ]
        positions = [item.get("position") for item in motor_positions]
        response = self._ros_service(
            "/apply_joint_trajectory",
            "datatypes/srv/ApplyJointTrajectory",
            {
                "joint_trajectory": {
                    "joint_names": names,
                    "points": [{"positions": [position]} for position in positions],
                }
            },
        )
        if not response.get("successful"):
            raise BackendError("pose_failed", f"pose {pose_name!r} was not applied")
        return {"poseName": pose_name, "motorCount": len(names)}

    def run_program(self, program_id: str) -> dict[str, Any]:
        response = self._ros_service(
            "/proxy_run_program_start",
            "datatypes/srv/ProxyRunProgramStart",
            {"program_number": program_id},
        )
        goal_id = response.get("proxy_goal_id", response.get("proxyGoalId"))
        if not goal_id:
            raise BackendError("program_failed", f"program {program_id!r} did not start")
        return {"programId": program_id, "goalId": goal_id}

    def set_led(
        self, button_id: int, red: int, green: int, blue: int
    ) -> dict[str, Any]:
        response = self._ros_service(
            "/tf_button/set_color",
            "button_service/srv/SetButtonColor",
            {
                "button_id": button_id,
                "red": red,
                "green": green,
                "blue": blue,
            },
        )
        if not response.get("success"):
            raise BackendError(
                "led_failed", response.get("message") or "LED color was not changed"
            )
        return {
            "buttonId": button_id,
            "color": {"red": red, "green": green, "blue": blue},
        }

    def set_relay(self, turned_on: bool) -> dict[str, Any]:
        response = self._ros_service(
            "/set_solid_state_relay_state",
            "datatypes/srv/SetSolidStateRelay",
            {"solid_state_relay_state": {"turned_on": turned_on}},
        )
        if not response.get("successful"):
            raise BackendError("relay_failed", "solid-state relay was not changed")
        return {"turnedOn": turned_on}

    def soul_append(self, personality_id: str, lesson: str) -> dict[str, Any]:
        personality = self._http(
            "POST",
            f"/voice-assistant/personality/{quote(personality_id, safe='')}/soul/append",
            {"lesson": lesson},
        )
        return {
            "personalityId": personality_id,
            "appended": True,
            "personality": personality,
        }
