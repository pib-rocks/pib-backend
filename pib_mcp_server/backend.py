"""Adapters from MCP tools to pib's HTTP API and ROS 2 services."""

from __future__ import annotations

import hashlib
import json
import math
import os
import time
import uuid
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.parse import quote
from urllib.request import Request, urlopen

MOTOR_POSITION_UNIT = "degree"


def _centidegrees_to_degrees(value: int | float | None) -> float | None:
    if value is None:
        return None
    return float(value) / 100.0


def _degrees_to_radians(degrees: float) -> float:
    return math.radians(degrees)


def _duration_from_seconds(seconds: float) -> dict[str, int]:
    whole_seconds = int(seconds)
    nanoseconds = int(round((seconds - whole_seconds) * 1_000_000_000))
    if nanoseconds >= 1_000_000_000:
        whole_seconds += 1
        nanoseconds -= 1_000_000_000
    return {"sec": whole_seconds, "nanosec": nanoseconds}


def _compact_motor(motor: dict[str, Any]) -> dict[str, Any]:
    name = motor.get("name")
    minimum = motor.get("rotationRangeMin", motor.get("rotation_range_min"))
    maximum = motor.get("rotationRangeMax", motor.get("rotation_range_max"))
    return {
        "id": name,
        "name": name,
        "position": _centidegrees_to_degrees(motor.get("currentPosition")),
        "limits": {
            "min": _centidegrees_to_degrees(minimum),
            "max": _centidegrees_to_degrees(maximum),
        },
        "unit": MOTOR_POSITION_UNIT,
    }


def _compact_diagnostics(summary: dict[str, Any]) -> dict[str, Any]:
    keys = (
        "overallStatus",
        "status",
        "hardwareVariant",
        "cpuStatus",
        "memoryStatus",
        "diskStatus",
        "containersStatus",
        "brickletsStatus",
        "controllersStatus",
        "cpuTemperature",
        "motorsOnline",
        "healthyContainersCount",
        "totalContainersCount",
    )
    return {key: summary[key] for key in keys if key in summary}


def _compact_pose(pose: dict[str, Any]) -> dict[str, Any]:
    return {
        "poseId": pose.get("poseId", pose.get("pose_id")),
        "name": pose.get("name"),
        "deletable": pose.get("deletable"),
    }


def _compact_program(program: dict[str, Any]) -> dict[str, Any]:
    return {
        "name": program.get("name"),
        "programNumber": program.get("programNumber", program.get("program_number")),
    }


def _compact_motion_model(model: dict[str, Any]) -> dict[str, Any]:
    compact: dict[str, Any] = {"schemaVersion": model.get("schemaVersion", 1)}
    if "units" in model:
        compact["units"] = model["units"]
    if "prompt" in model:
        compact["prompt"] = model["prompt"]
    return compact


def _compact_smarthome_device(device: dict[str, Any]) -> dict[str, Any]:
    return {
        key: device[key]
        for key in (
            "entityId",
            "friendlyName",
            "room",
            "domain",
            "state",
            "exposeToVoice",
        )
        if key in device
    }


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
            rosbridge_url or os.getenv("PIB_MCP_ROSBRIDGE_URL") or "ws://localhost:9090"
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
            raise BackendError(
                "api_unavailable", f"pib API request failed: {exc}"
            ) from exc

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
                "ros_timeout",
                f"ROS service {service} timed out after {self.timeout:g}s",
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

    def _motors_with_live_positions(self) -> list[dict[str, Any]]:
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
                raw_position = (
                    position.get("position") if position.get("successful") else None
                )
                motor["currentPosition"] = (
                    _centidegrees_to_degrees(raw_position)
                    if raw_position is not None
                    else None
                )
                if not position.get("successful"):
                    motor["positionError"] = position.get("message", "unavailable")
            except BackendError as exc:
                motor["currentPosition"] = None
                motor["positionError"] = exc.message
        return motors

    def list_motors(self, detail: bool = False) -> dict[str, Any]:
        motors = self._motors_with_live_positions()
        if detail:
            bricklets = self._http("GET", "/bricklet").get("bricklets", [])
            return {"motors": motors, "bricklets": bricklets}
        return {"motors": [_compact_motor(motor) for motor in motors]}

    def get_state(self, detail: bool = False) -> dict[str, Any]:
        diagnostics = self._http("GET", "/diagnostics/summary")
        if detail:
            state = self.list_motors(detail=True)
            state["diagnostics"] = diagnostics
            return state
        return {
            "motors": [
                _compact_motor(motor) for motor in self._motors_with_live_positions()
            ],
            "diagnostics": _compact_diagnostics(diagnostics),
        }

    def get_motion_model(self, detail: bool = False) -> dict[str, Any]:
        model = self._http("GET", "/system/motion-model")
        if detail:
            return model
        return _compact_motion_model(model)

    def list_poses(self, detail: bool = False) -> list[dict[str, Any]]:
        poses = list(self._http("GET", "/pose").get("poses", []))
        if detail:
            detailed: list[dict[str, Any]] = []
            for pose in poses:
                name = pose.get("name")
                if not name:
                    detailed.append(pose)
                    continue
                detailed.append(
                    self._http("GET", f"/pose/by-name/{quote(name, safe='')}")
                )
            return detailed
        return [_compact_pose(pose) for pose in poses]

    def list_programs(self, detail: bool = False) -> list[dict[str, Any]]:
        programs = list(self._http("GET", "/program").get("programs", []))
        if not detail:
            return [_compact_program(program) for program in programs]
        detailed: list[dict[str, Any]] = []
        for program in programs:
            number = program.get("programNumber", program.get("program_number"))
            if not number:
                detailed.append(program)
                continue
            code = self._http("GET", f"/program/{quote(str(number), safe='')}/code")
            detailed.append({**program, **code})
        return detailed

    def capture_image(self, detail: bool = False) -> dict[str, Any]:
        response = self._ros_service(
            "/get_camera_image", "datatypes/srv/GetCameraImage", {}
        )
        image = response.get("image_base64", response.get("imageBase64", ""))
        if not image:
            raise BackendError("camera_empty", "camera returned no image")
        if detail:
            return {"imageBase64": image, "mimeType": "image/jpeg"}
        digest = hashlib.sha256(image.encode("utf-8")).hexdigest()
        return {
            "imageRef": digest[:16],
            "mimeType": "image/jpeg",
            "byteLength": len(image.encode("utf-8")),
            "sha256": digest,
        }

    def _motor_result_rows(
        self,
        response: dict[str, Any],
        joint_names: list[str],
        position_degrees: list[float],
    ) -> list[dict[str, Any]]:
        raw_results = response.get("results")
        if isinstance(raw_results, list) and raw_results:
            rows: list[dict[str, Any]] = []
            for item in raw_results:
                motor_name = item.get("motor_name", item.get("motorName", ""))
                matching_degree = next(
                    (
                        deg
                        for name, deg in zip(joint_names, position_degrees)
                        if name == motor_name
                    ),
                    None,
                )
                rows.append(
                    {
                        "motorName": motor_name,
                        "positionDegrees": matching_degree,
                        "successful": bool(item.get("successful")),
                        "errorCode": item.get("error_code", item.get("errorCode", "")),
                        "detail": item.get("detail", ""),
                    }
                )
            return rows

        overall = bool(response.get("successful"))
        return [
            {
                "motorName": name,
                "positionDegrees": degree,
                "successful": overall,
                "errorCode": "" if overall else "motion_failed",
                "detail": "" if overall else "joint trajectory was not applied",
            }
            for name, degree in zip(joint_names, position_degrees)
        ]

    def _apply_joint_trajectory_degrees(
        self,
        joint_names: list[str],
        points_degrees: list[dict[str, Any]],
    ) -> dict[str, Any]:
        if not joint_names:
            raise BackendError("invalid_trajectory", "joint_names must not be empty")
        if not points_degrees:
            raise BackendError(
                "invalid_trajectory", "trajectory must have at least one point"
            )

        ros_points: list[dict[str, Any]] = []
        for point in points_degrees:
            positions_degrees = list(point.get("positions_degrees", []))
            if len(positions_degrees) != len(joint_names):
                raise BackendError(
                    "invalid_trajectory",
                    (
                        f"trajectory point has {len(positions_degrees)} positions for "
                        f"{len(joint_names)} joint names"
                    ),
                )
            ros_point: dict[str, Any] = {
                "positions": [
                    _degrees_to_radians(float(value)) for value in positions_degrees
                ],
            }
            time_from_start_sec = point.get("time_from_start_sec")
            if time_from_start_sec is not None:
                ros_point["time_from_start"] = _duration_from_seconds(
                    float(time_from_start_sec)
                )
            ros_points.append(ros_point)

        response = self._ros_service(
            "/apply_joint_trajectory",
            "datatypes/srv/ApplyJointTrajectory",
            {
                "joint_trajectory": {
                    "joint_names": joint_names,
                    "points": ros_points,
                }
            },
        )
        target_degrees = list(points_degrees[-1]["positions_degrees"])
        results = self._motor_result_rows(response, joint_names, target_degrees)
        return {
            "successful": bool(response.get("successful")),
            "results": results,
        }

    def move_motors(self, moves: list[dict[str, Any]]) -> dict[str, Any]:
        if not moves:
            raise BackendError("invalid_parameter", "moves must not be empty")
        joint_names = [move["motor"] for move in moves]
        position_degrees = [float(move["position_degrees"]) for move in moves]
        outcome = self._apply_joint_trajectory_degrees(
            joint_names,
            [{"positions_degrees": position_degrees}],
        )
        if not outcome["successful"]:
            failed = [row for row in outcome["results"] if not row["successful"]]
            raise BackendError(
                "motion_failed",
                "one or more motors did not move",
                {"results": failed},
            )
        return {"results": outcome["results"]}

    def apply_joint_trajectory(
        self,
        joint_names: list[str],
        points: list[dict[str, Any]],
    ) -> dict[str, Any]:
        outcome = self._apply_joint_trajectory_degrees(joint_names, points)
        if not outcome["successful"]:
            failed = [row for row in outcome["results"] if not row["successful"]]
            raise BackendError(
                "trajectory_failed",
                "joint trajectory was not fully applied",
                {"results": failed},
            )
        return outcome

    def move_motor(self, motor_name: str, position_degrees: float) -> dict[str, Any]:
        outcome = self.move_motors(
            [{"motor": motor_name, "position_degrees": position_degrees}]
        )
        row = outcome["results"][0]
        return {
            "motorName": row["motorName"],
            "positionDegrees": row["positionDegrees"],
            "successful": row["successful"],
            "errorCode": row["errorCode"],
            "detail": row["detail"],
        }

    def apply_pose(self, pose_name: str) -> dict[str, Any]:
        pose = self._http("GET", f"/pose/by-name/{quote(pose_name, safe='')}")
        motor_positions = pose.get("motorPositions", pose.get("motor_positions", []))
        if not motor_positions:
            raise BackendError(
                "pose_empty", f"pose {pose_name!r} has no motor positions"
            )
        names = [
            item.get("motorName", item.get("motor_name")) for item in motor_positions
        ]
        positions_degrees: list[float] = []
        for item in motor_positions:
            position = _centidegrees_to_degrees(item.get("position"))
            if position is None:
                raise BackendError(
                    "pose_invalid",
                    f"pose {pose_name!r} has missing motor position values",
                )
            positions_degrees.append(position)
        outcome = self._apply_joint_trajectory_degrees(
            names,
            [{"positions_degrees": positions_degrees}],
        )
        if not outcome["successful"]:
            failed = [row for row in outcome["results"] if not row["successful"]]
            raise BackendError(
                "pose_failed",
                f"pose {pose_name!r} was not fully applied",
                {"results": failed},
            )
        return {
            "poseName": pose_name,
            "motorCount": len(names),
            "results": outcome["results"],
        }

    def run_program(self, program_id: str) -> dict[str, Any]:
        response = self._ros_service(
            "/proxy_run_program_start",
            "datatypes/srv/ProxyRunProgramStart",
            {"program_number": program_id},
        )
        goal_id = response.get("proxy_goal_id", response.get("proxyGoalId"))
        if not goal_id:
            raise BackendError(
                "program_failed", f"program {program_id!r} did not start"
            )
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

    def _smarthome_devices_path(
        self,
        *,
        room: str | None = None,
        domain: str | None = None,
        voice_only: bool = False,
    ) -> str:
        query: list[str] = []
        if voice_only:
            query.append("voiceOnly=true")
        if room:
            query.append(f"room={quote(room, safe='')}")
        if domain:
            query.append(f"domain={quote(domain, safe='')}")
        suffix = "?" + "&".join(query) if query else ""
        return "/smart-home/devices" + suffix

    @staticmethod
    def _resolve_smarthome_device(
        target: str, devices: list[dict[str, Any]]
    ) -> dict[str, Any] | None:
        stripped = target.strip()
        if not stripped:
            return None
        lowered = stripped.casefold()
        for device in devices:
            entity_id = device.get("entityId", "")
            friendly = device.get("friendlyName", "")
            if entity_id == stripped or friendly == stripped:
                return device
            if friendly.casefold() == lowered:
                return device

        matches = [
            item
            for item in devices
            if lowered in item.get("entityId", "").casefold()
            or lowered in item.get("friendlyName", "").casefold()
            or lowered
            in f"{item.get('room', '')} {item.get('friendlyName', '')}".casefold()
        ]
        if len(matches) == 1:
            return matches[0]
        return None

    def list_smarthome_devices(
        self,
        room: str | None = None,
        domain: str | None = None,
        detail: bool = False,
    ) -> dict[str, Any]:
        path = self._smarthome_devices_path(
            room=(room or None), domain=(domain or None), voice_only=True
        )
        payload = self._http("GET", path)
        devices = list(payload.get("devices", []))
        if detail:
            return {"devices": devices}
        return {"devices": [_compact_smarthome_device(device) for device in devices]}

    def get_smarthome_state(self, target: str, detail: bool = False) -> dict[str, Any]:
        devices = self.list_smarthome_devices(detail=True)["devices"]
        device = self._resolve_smarthome_device(target, devices)
        if device is None:
            raise BackendError(
                "device_not_found", f"unknown SmartHome device {target!r}"
            )
        if detail:
            return device
        return _compact_smarthome_device(device)

    def control_smarthome_device(
        self, target: str, action: str, value: Any = None
    ) -> dict[str, Any]:
        all_devices = list(self._http("GET", "/smart-home/devices").get("devices", []))
        resolved = self._resolve_smarthome_device(target, all_devices)
        if resolved is not None and not resolved.get("exposeToVoice", False):
            raise BackendError(
                "voice_access_denied",
                f"SmartHome device {target!r} is not enabled for voice control",
            )
        entity_id = resolved.get("entityId", target) if resolved is not None else target
        body: dict[str, Any] = {"action": action}
        if value is not None:
            body["value"] = value
        return self._http(
            "POST",
            f"/smart-home/devices/{quote(entity_id, safe='')}/action",
            body,
        )

    def trigger_smarthome_scene(self, scene_name: str) -> dict[str, Any]:
        return self.control_smarthome_device(scene_name, "activate")

    def soul_append(
        self, personality_id: str, lesson: str, detail: bool = False
    ) -> dict[str, Any]:
        personality = self._http(
            "POST",
            f"/voice-assistant/personality/{quote(personality_id, safe='')}/soul/append",
            {"lesson": lesson},
        )
        result: dict[str, Any] = {
            "personalityId": personality_id,
            "appended": True,
        }
        if detail:
            result["personality"] = personality
        return result
