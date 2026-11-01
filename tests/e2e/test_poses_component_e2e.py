"""Live E2E suite for the Poses component (/pose) against a real robot (PR-2160).

Verifies that clicking the Apply button on a pose in Cerebra:
1. Sends a valid single-point JointTrajectory over rosbridge to /apply_joint_trajectory
   (not the retired legacy N-points-of-1-position zip shape).
2. Receives a successful service response (successful=True, no legacy_shape_rejected).
3. Physically drives the robot's joint to the pose's target position.
"""

from __future__ import annotations

import base64
import json
import os
import socket
import time
from typing import Any

import pytest
import requests
from playwright.sync_api import expect, sync_playwright

from robot_address import api_url, robot_base_url, robot_hostname

ROBOT_URL = robot_base_url()
API_URL = api_url()
ROBOT_HOST = robot_hostname()
ROSBRIDGE_PORT = int(os.getenv("PIB_ROSBRIDGE_PORT", "9090"))
REQUEST_TIMEOUT_S = 10
POSITION_TOLERANCE_CENTIDEG = 350  # 3.5 deg tolerance for live servo settling


def _chromium_executable() -> str | None:
    override = os.getenv("PIB_E2E_CHROMIUM")
    if override and os.path.exists(override):
        return override
    for path in (
        "/home/pib/.cache/ms-playwright/chromium-1248/chrome-linux64/chrome",
        "/home/pib/.cache/ms-playwright/chromium-1234/chrome-linux64/chrome",
        "/home/pib/.cache/ms-playwright/chromium-1228/chrome-linux64/chrome",
        "/usr/bin/chromium",
        "/usr/bin/chromium-browser",
    ):
        if os.path.exists(path):
            return path
    return None


def _rosbridge_call_service(
    service: str,
    service_type: str,
    args: dict[str, Any],
    timeout: float = 5.0,
) -> dict[str, Any]:
    """Call a ROS 2 service over rosbridge WebSocket using stdlib sockets."""
    sock = socket.create_connection((ROBOT_HOST, ROSBRIDGE_PORT), timeout=timeout)
    try:
        key = base64.b64encode(os.urandom(16)).decode("ascii")
        handshake = (
            f"GET / HTTP/1.1\r\n"
            f"Host: {ROBOT_HOST}:{ROSBRIDGE_PORT}\r\n"
            f"Upgrade: websocket\r\n"
            f"Connection: Upgrade\r\n"
            f"Sec-WebSocket-Key: {key}\r\n"
            f"Sec-WebSocket-Version: 13\r\n\r\n"
        )
        sock.sendall(handshake.encode("ascii"))
        resp = b""
        while b"\r\n\r\n" not in resp:
            chunk = sock.recv(4096)
            if not chunk:
                raise RuntimeError("WebSocket handshake closed unexpectedly")
            resp += chunk

        payload = json.dumps(
            {
                "op": "call_service",
                "id": f"e2e_{int(time.time() * 1000)}",
                "service": service,
                "type": service_type,
                "args": args,
            }
        ).encode("utf-8")

        mask = os.urandom(4)
        header = bytearray([0x81])
        length = len(payload)
        if length < 126:
            header.append(0x80 | length)
        elif length < 65536:
            header.append(0x80 | 126)
            header.extend(length.to_bytes(2, "big"))
        else:
            header.append(0x80 | 127)
            header.extend(length.to_bytes(8, "big"))
        header.extend(mask)
        masked = bytearray(b ^ mask[i % 4] for i, b in enumerate(payload))
        sock.sendall(header + masked)

        first2 = sock.recv(2)
        ln = first2[1] & 0x7F
        if ln == 126:
            ln = int.from_bytes(sock.recv(2), "big")
        elif ln == 127:
            ln = int.from_bytes(sock.recv(8), "big")
        body = b""
        while len(body) < ln:
            body += sock.recv(ln - len(body))
        return json.loads(body.decode("utf-8"))
    finally:
        sock.close()


def _read_live_joint_centideg(joint_name: str) -> int:
    msg = _rosbridge_call_service(
        "/get_joint_position",
        "datatypes/srv/GetJointPosition",
        {"joint_name": joint_name},
    )
    assert msg.get("result") is True, f"rosbridge /get_joint_position failed: {msg}"
    values = msg.get("values") or {}
    assert values.get("successful") is True, f"GetJointPosition failed: {msg}"
    return int(values["position"])


def _startup_pose_motor_positions() -> list[dict[str, Any]]:
    poses_resp = requests.get(f"{API_URL}/pose", timeout=REQUEST_TIMEOUT_S)
    poses_resp.raise_for_status()
    poses = poses_resp.json().get("poses", [])
    startup = next((p for p in poses if p["name"] == "Startup/Resting"), poses[0])
    mp_resp = requests.get(
        f"{API_URL}/pose/{startup['poseId']}/motor-positions",
        timeout=REQUEST_TIMEOUT_S,
    )
    mp_resp.raise_for_status()
    return mp_resp.json()["motorPositions"]


class TestPosesComponentE2E:
    """End-to-end verification of applying poses in the Cerebra Poses view (/pose)."""

    def test_apply_pose_moves_robot_and_sends_standard_joint_trajectory(self):
        base_positions = _startup_pose_motor_positions()
        assert (
            len(base_positions) >= 2
        ), f"Expected multi-motor pose, got {base_positions}"

        initial_head = _read_live_joint_centideg("turn_head_motor")
        # Pick a safe target for turn_head_motor at least 20 deg (2000 centideg) away from current
        target_head = -2500 if initial_head >= 0 else 2500

        pose_name = f"e2e-apply-pose-{int(time.time())}"
        test_positions = [
            {
                "motorName": item["motorName"],
                "position": (
                    target_head
                    if item["motorName"] == "turn_head_motor"
                    else item["position"]
                ),
            }
            for item in base_positions
        ]

        created_pose_id: str | None = None
        executable = _chromium_executable()

        try:
            create_resp = requests.post(
                f"{API_URL}/pose",
                json={"name": pose_name, "motorPositions": test_positions},
                timeout=REQUEST_TIMEOUT_S,
            )
            create_resp.raise_for_status()
            created_pose_id = create_resp.json()["poseId"]

            with sync_playwright() as p:
                launch_kwargs: dict[str, Any] = {
                    "headless": True,
                    "args": ["--no-sandbox"],
                }
                if executable:
                    launch_kwargs["executable_path"] = executable
                browser = p.chromium.launch(**launch_kwargs)
                context = browser.new_context(viewport={"width": 1280, "height": 800})
                page = context.new_page()

                # Record rosbridge WebSocket frames for /apply_joint_trajectory
                ws_sent_trajectories: list[dict[str, Any]] = []
                ws_service_responses: list[dict[str, Any]] = []

                def _on_ws(ws):
                    def _on_sent(payload):
                        if (
                            isinstance(payload, str)
                            and "/apply_joint_trajectory" in payload
                        ):
                            try:
                                ws_sent_trajectories.append(json.loads(payload))
                            except Exception:
                                pass

                    def _on_received(payload):
                        if (
                            isinstance(payload, str)
                            and "apply_joint_trajectory" in payload
                        ):
                            try:
                                ws_service_responses.append(json.loads(payload))
                            except Exception:
                                pass

                    ws.on("framesent", _on_sent)
                    ws.on("framereceived", _on_received)

                page.on("websocket", _on_ws)

                try:
                    page.goto(
                        f"{ROBOT_URL}/pose",
                        wait_until="domcontentloaded",
                        timeout=30000,
                    )
                    apply_btn = page.locator(
                        f'[data-test="BTN_Pose_Apply_{pose_name}"]'
                    )
                    expect(apply_btn).to_be_visible(timeout=15000)
                    expect(apply_btn).to_be_enabled(timeout=10000)

                    # Wait briefly so rosbridge WebSocket is connected
                    page.wait_for_timeout(1500)

                    apply_btn.click()

                    # Wait for the /apply_joint_trajectory WebSocket call and physical movement
                    deadline = time.monotonic() + 6.0
                    while time.monotonic() < deadline and not ws_service_responses:
                        page.wait_for_timeout(200)

                    assert (
                        ws_sent_trajectories
                    ), "Clicking Apply Pose did not send an /apply_joint_trajectory request over rosbridge"
                    sent_jt = ws_sent_trajectories[-1]["args"]["joint_trajectory"]
                    assert len(sent_jt["points"]) == 1, (
                        f"Expected standard 1-point JointTrajectory, but Cerebra sent "
                        f"{len(sent_jt['points'])} points (legacy zip shape): {sent_jt}"
                    )
                    assert len(sent_jt["points"][0]["positions"]) == len(
                        sent_jt["joint_names"]
                    ), f"Expected positions count to match joint_names count: {sent_jt}"

                    assert (
                        ws_service_responses
                    ), "No service_response received from rosbridge for /apply_joint_trajectory"
                    last_resp = ws_service_responses[-1]
                    values = last_resp.get("values") or {}
                    assert (
                        values.get("successful") is True
                    ), f"/apply_joint_trajectory rejected the pose: {last_resp}"

                    # Poll live servo position until it reaches target_head within tolerance
                    reached = False
                    last_measured = initial_head
                    move_deadline = time.monotonic() + 5.0
                    while time.monotonic() < move_deadline:
                        last_measured = _read_live_joint_centideg("turn_head_motor")
                        if (
                            abs(last_measured - target_head)
                            <= POSITION_TOLERANCE_CENTIDEG
                        ):
                            reached = True
                            break
                        time.sleep(0.15)

                    assert reached, (
                        f"turn_head_motor did not move to pose target {target_head} cdeg "
                        f"(initial={initial_head} cdeg, last_measured={last_measured} cdeg)"
                    )
                finally:
                    context.close()
                    browser.close()
        finally:
            if created_pose_id:
                try:
                    requests.delete(
                        f"{API_URL}/pose/{created_pose_id}", timeout=REQUEST_TIMEOUT_S
                    )
                except Exception:
                    pass
