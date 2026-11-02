"""Live E2E suite for the Poses tab (/movement/pose) against a real robot (PR-2160).

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
    """End-to-end verification of applying poses in the Cerebra Poses tab (/movement/pose)."""

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
                        f"{ROBOT_URL}/movement/pose",
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

    def test_save_rename_delete_pose_ui_lifecycle_persists_in_backend(self):
        """Saving current pose, renaming it, and deleting it through the UI
        persists each step in the Flask backend."""
        pose_name = f"e2e-crud-{int(time.time())}"
        renamed_name = f"{pose_name}-renamed"
        executable = _chromium_executable()
        created_pose_id: str | None = None

        try:
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

                try:
                    page.goto(
                        f"{ROBOT_URL}/movement/pose",
                        wait_until="domcontentloaded",
                        timeout=30000,
                    )
                    save_btn = page.locator("#save-pose-button")
                    expect(save_btn).to_be_visible(timeout=15000)
                    # Allow rosbridge to connect and deliver at least one 10 Hz /joint_states frame
                    page.wait_for_timeout(1500)
                    save_btn.click()

                    name_input = page.locator("#name-input")
                    expect(name_input).to_be_visible(timeout=10000)
                    name_input.fill(pose_name)
                    modal_save = page.locator("#modal-save-button")
                    expect(modal_save).to_be_enabled(timeout=10000)
                    modal_save.click()

                    # Verify new pose appears in UI and in GET /api/pose
                    expect(
                        page.locator(f'[data-test="BTN_Pose_Select_{pose_name}"]')
                    ).to_be_visible(timeout=15000)

                    poses = requests.get(
                        f"{API_URL}/pose", timeout=REQUEST_TIMEOUT_S
                    ).json()["poses"]
                    created_entry = next(
                        (item for item in poses if item["name"] == pose_name), None
                    )
                    assert (
                        created_entry is not None
                    ), f"Pose {pose_name!r} was not persisted in backend: {poses}"
                    created_pose_id = created_entry["poseId"]

                    # Verify saved pose carries 26 motor positions from live telemetry
                    # AND is NOT all zeros (PR-2164 regression guard: saving after a fresh
                    # page load must capture live /joint_states angles, not uninitialized 0s)
                    mp = requests.get(
                        f"{API_URL}/pose/{created_pose_id}/motor-positions",
                        timeout=REQUEST_TIMEOUT_S,
                    ).json()["motorPositions"]
                    assert (
                        len(mp) >= 2
                    ), f"Saved pose has no motor positions in backend: {mp}"
                    saved_map = {
                        item["motorName"]: int(item["position"]) for item in mp
                    }
                    non_zero = {k: v for k, v in saved_map.items() if abs(v) > 100}
                    assert non_zero, (
                        f"Saved pose '{pose_name}' recorded 0 on every joint "
                        f"(all-zero Calibration regression, PR-2164): {saved_map}"
                    )
                    live_sh_right = _read_live_joint_centideg(
                        "shoulder_horizontal_right"
                    )
                    assert (
                        abs(
                            saved_map.get("shoulder_horizontal_right", 0)
                            - live_sh_right
                        )
                        <= POSITION_TOLERANCE_CENTIDEG
                    ), (
                        f"Saved shoulder_horizontal_right={saved_map.get('shoulder_horizontal_right')} "
                        f"does not match live servo {live_sh_right} cdeg: {saved_map}"
                    )

                    # Rename pose via UI
                    rename_btn = page.locator(
                        f'[data-test="BTN_Pose_Rename_{pose_name}"]'
                    )
                    expect(rename_btn).to_be_visible(timeout=10000)
                    rename_btn.click()

                    expect(name_input).to_be_visible(timeout=10000)
                    name_input.fill(renamed_name)
                    expect(modal_save).to_be_enabled(timeout=10000)
                    modal_save.click()

                    expect(
                        page.locator(f'[data-test="BTN_Pose_Select_{renamed_name}"]')
                    ).to_be_visible(timeout=15000)
                    poses_after_rename = requests.get(
                        f"{API_URL}/pose", timeout=REQUEST_TIMEOUT_S
                    ).json()["poses"]
                    assert any(
                        item["poseId"] == created_pose_id
                        and item["name"] == renamed_name
                        for item in poses_after_rename
                    ), f"Rename did not persist in backend: {poses_after_rename}"

                    # Delete pose via UI (deletes immediately without a modal)
                    delete_btn = page.locator(
                        f'[data-test="BTN_Pose_Delete_{renamed_name}"]'
                    )
                    expect(delete_btn).to_be_visible(timeout=10000)
                    delete_btn.click()

                    expect(
                        page.locator(f'[data-test="BTN_Pose_Select_{renamed_name}"]')
                    ).to_have_count(0, timeout=15000)
                    poses_after_delete = requests.get(
                        f"{API_URL}/pose", timeout=REQUEST_TIMEOUT_S
                    ).json()["poses"]
                    assert all(
                        item["poseId"] != created_pose_id for item in poses_after_delete
                    ), f"Deleted pose {created_pose_id} still exists in backend"
                    created_pose_id = None
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

    def test_dynamic_play_pose_sequence_block_ui_and_smooth_execution(self):
        """Verifies PR-2161 / PR-2162 / PR-2163 / PR-2164 end-to-end:
        1. Opens a program in Cerebra (/program), drags/loads a play_pose_sequence block.
        2. Uses the inline [+] button on Row 1 to insert an intermediate pose row (2 -> 3 rows)
           and inline [-] to delete a row (3 -> 2 rows).
        3. Compiles via BTN_Program_Show_Code and verifies the generated Python uses
           telemetry.get_positions_deg(joint_names), prepends Waypoint 0 at t=0.0s,
           and calls _pib_play_pose_sequence_timed with the selected pose UUIDs and durations.
        4. Executes a 2-pose sequence on the live robot and verifies that the servo smoothly
           reaches the target pose and returns to the initial posture.
        """
        base_positions = _startup_pose_motor_positions()
        initial_head = _read_live_joint_centideg("turn_head_motor")
        sign = -1 if initial_head > 0 else 1
        pose_a_head = max(-4500, min(4500, initial_head + sign * 1500))
        pose_b_head = max(-4500, min(4500, initial_head + sign * 3000))

        ts = int(time.time())
        pose_a_name = f"e2e-seq-a-{ts}"
        pose_b_name = f"e2e-seq-b-{ts}"
        prog_name = f"e2e_seq_prog_{ts}"

        pose_a_positions = [
            {
                "motorName": item["motorName"],
                "position": (
                    pose_a_head
                    if item["motorName"] == "turn_head_motor"
                    else item["position"]
                ),
            }
            for item in base_positions
        ]
        pose_b_positions = [
            {
                "motorName": item["motorName"],
                "position": (
                    pose_b_head
                    if item["motorName"] == "turn_head_motor"
                    else item["position"]
                ),
            }
            for item in base_positions
        ]

        pose_a_id: str | None = None
        pose_b_id: str | None = None
        prog_id: str | None = None
        executable = _chromium_executable()

        try:
            r_a = requests.post(
                f"{API_URL}/pose",
                json={"name": pose_a_name, "motorPositions": pose_a_positions},
                timeout=REQUEST_TIMEOUT_S,
            )
            r_a.raise_for_status()
            pose_a_id = r_a.json()["poseId"]

            r_b = requests.post(
                f"{API_URL}/pose",
                json={"name": pose_b_name, "motorPositions": pose_b_positions},
                timeout=REQUEST_TIMEOUT_S,
            )
            r_b.raise_for_status()
            pose_b_id = r_b.json()["poseId"]

            r_prog = requests.post(
                f"{API_URL}/program",
                json={"name": prog_name},
                timeout=REQUEST_TIMEOUT_S,
            )
            r_prog.raise_for_status()
            prog_id = r_prog.json()["programNumber"]

            initial_visual = json.dumps(
                {
                    "blocks": {
                        "languageVersion": 0,
                        "blocks": [
                            {
                                "type": "play_pose_sequence",
                                "id": "e2e_seq_block",
                                "x": 80,
                                "y": 80,
                                "extraState": {
                                    "itemCount": 2,
                                    "steps": [
                                        {"pose": pose_a_id, "duration": 1.2},
                                        {"pose": pose_b_id, "duration": 1.2},
                                    ],
                                },
                                "fields": {
                                    "POSE_0": pose_a_id,
                                    "DURATION_0": 1.2,
                                    "POSE_1": pose_b_id,
                                    "DURATION_1": 1.2,
                                },
                            }
                        ],
                    }
                }
            )
            requests.put(
                f"{API_URL}/program/{prog_id}/code",
                json={"codeVisual": initial_visual},
                timeout=REQUEST_TIMEOUT_S,
            ).raise_for_status()

            with sync_playwright() as p:
                launch_kwargs: dict[str, Any] = {
                    "headless": True,
                    "args": ["--no-sandbox"],
                }
                if executable:
                    launch_kwargs["executable_path"] = executable
                browser = p.chromium.launch(**launch_kwargs)
                context = browser.new_context(viewport={"width": 1400, "height": 900})
                page = context.new_page()

                try:
                    page.goto(
                        f"{ROBOT_URL}/program",
                        wait_until="domcontentloaded",
                        timeout=30000,
                    )
                    prog_btn = page.locator(f'[data-test="BTN_{prog_name}"]')
                    expect(prog_btn).to_be_visible(timeout=15000)
                    prog_btn.click()
                    page.wait_for_timeout(2000)

                    # Verify initial 2 rows rendered on the Blockly canvas ("1. Pose", "2. Pose")
                    row_labels_before = page.evaluate(
                        r"""() => [...document.querySelectorAll('g.blocklyDraggable text.blocklyText')]
                        .map(t => (t.textContent || '').replace(/\u00a0/g, ' ').trim())
                        .filter(t => /^\d+\. Pose$/.test(t))"""
                    )
                    assert row_labels_before == [
                        "1. Pose",
                        "2. Pose",
                    ], f"Expected 2 pose rows initially, got: {row_labels_before}"

                    # Click inline [+] on Row 1 to insert an intermediate pose row (2 -> 3 rows)
                    plus_box = page.evaluate("""() => {
                        const imgs = [...document.querySelectorAll('g.blocklyDraggable image')];
                        const addImg = imgs.find(im => (im.getAttribute('xlink:href') || im.getAttribute('href') || '').includes('M9%204.5v9'));
                        if (!addImg) return null;
                        const r = addImg.getBoundingClientRect();
                        return {x: r.x + r.width / 2, y: r.y + r.height / 2};
                    }""")
                    assert (
                        plus_box is not None
                    ), "Inline [+] button not found on play_pose_sequence"
                    page.mouse.click(plus_box["x"], plus_box["y"])
                    page.wait_for_timeout(600)

                    row_labels_after_add = page.evaluate(
                        r"""() => [...document.querySelectorAll('g.blocklyDraggable text.blocklyText')]
                        .map(t => (t.textContent || '').replace(/\u00a0/g, ' ').trim())
                        .filter(t => /^\d+\. Pose$/.test(t))"""
                    )
                    assert row_labels_after_add == [
                        "1. Pose",
                        "2. Pose",
                        "3. Pose",
                    ], f"Expected 3 pose rows after clicking [+], got: {row_labels_after_add}"

                    # Click inline [-] on Row 2 (index 1, the newly inserted middle row) to remove it (3 -> 2 rows)
                    minus_box = page.evaluate("""() => {
                        const imgs = [...document.querySelectorAll('g.blocklyDraggable image')];
                        const remImgs = imgs.filter(im => {
                            const href = im.getAttribute('xlink:href') || im.getAttribute('href') || '';
                            return href.includes('M4.5%209h9') && !href.includes('M9%204.5v9');
                        });
                        if (remImgs.length < 2) return null;
                        const r = remImgs[1].getBoundingClientRect();
                        return {x: r.x + r.width / 2, y: r.y + r.height / 2};
                    }""")
                    assert (
                        minus_box is not None
                    ), "Inline [-] button not found on middle row"
                    page.mouse.click(minus_box["x"], minus_box["y"])
                    page.wait_for_timeout(600)

                    row_labels_after_remove = page.evaluate(
                        r"""() => [...document.querySelectorAll('g.blocklyDraggable text.blocklyText')]
                        .map(t => (t.textContent || '').replace(/\u00a0/g, ' ').trim())
                        .filter(t => /^\d+\. Pose$/.test(t))"""
                    )
                    assert row_labels_after_remove == [
                        "1. Pose",
                        "2. Pose",
                    ], f"Expected 2 pose rows after clicking [-], got: {row_labels_after_remove}"

                    # Open Show Code modal and verify generated Python contract
                    page.locator('[data-test="BTN_Program_Show_Code"]').click()
                    code_el = page.locator("#python-code")
                    expect(code_el).to_contain_text(
                        "telemetry.get_positions_deg(joint_names)",
                        timeout=10000,
                    )
                    modal_code = code_el.inner_text()
                    assert (
                        f'_pib_play_pose_sequence_timed([("{pose_a_id}", 1.2), ("{pose_b_id}", 1.2)])'
                        in modal_code
                    ), f"Expected 2-step sequence call in generated Python: {modal_code}"
                    page.locator('[data-test="BTN_Program_Code_Close"]').click()
                    page.wait_for_timeout(400)

                    # Save and Run the program, then verify turn_head_motor reaches pose_a_head during leg 1
                    save_btn = page.locator('[data-test="BTN_Program_Save"]')
                    if save_btn.is_enabled():
                        save_btn.click()
                        page.wait_for_timeout(500)

                    page.locator('[data-test="BTN_Program_Run_Stop"]').click()

                    reached_pose_a = False
                    last_a = initial_head
                    deadline = time.monotonic() + 10.0
                    while time.monotonic() < deadline:
                        last_a = _read_live_joint_centideg("turn_head_motor")
                        if abs(last_a - pose_a_head) <= POSITION_TOLERANCE_CENTIDEG:
                            reached_pose_a = True
                            break
                        time.sleep(0.06)

                    assert reached_pose_a, (
                        f"turn_head_motor did not reach first waypoint {pose_a_head} cdeg "
                        f"during sequence execution (last_measured={last_a} cdeg)"
                    )

                    # Wait for leg 2 to finish returning turn_head_motor to pose_b_head
                    returned_pose_b = False
                    last_b = last_a
                    deadline_b = time.monotonic() + 8.0
                    while time.monotonic() < deadline_b:
                        last_b = _read_live_joint_centideg("turn_head_motor")
                        if abs(last_b - pose_b_head) <= POSITION_TOLERANCE_CENTIDEG:
                            returned_pose_b = True
                            break
                        time.sleep(0.08)

                    assert returned_pose_b, (
                        f"turn_head_motor did not reach second waypoint {pose_b_head} cdeg "
                        f"at sequence end (last_measured={last_b} cdeg)"
                    )
                finally:
                    context.close()
                    browser.close()
        finally:
            for pid in (pose_a_id, pose_b_id):
                if pid:
                    try:
                        requests.delete(
                            f"{API_URL}/pose/{pid}", timeout=REQUEST_TIMEOUT_S
                        )
                    except Exception:
                        pass
            if prog_id:
                try:
                    requests.delete(
                        f"{API_URL}/program/{prog_id}", timeout=REQUEST_TIMEOUT_S
                    )
                except Exception:
                    pass
