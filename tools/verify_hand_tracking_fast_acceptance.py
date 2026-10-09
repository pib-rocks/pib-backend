#!/usr/bin/env python3
"""Post-merge acceptance for the durable hand_tracking_fast chain.

Local mode checks the repository source: device-safe FP16 reads, the
setuptools install path, and the host-queue inventory. It does not start a
model, recreate a container, or open a browser.

Live mode is opt-in. It talks to a rosbridge URL and, when asked, to the
local Docker daemon. It does not print credentials. Run it only after the
reviewed merge and the normal image update, with a real hand in view.
"""

from __future__ import annotations

import argparse
import ast
import json
import os
import subprocess
import sys
import time
import uuid
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
HAND_TRACKING = REPO_ROOT / "ros_packages/camera/oak_d_lite/hand_tracking.py"
CAMERA_SETUP = REPO_ROOT / "ros_packages/camera/setup.py"
INSTALLED_MODULE = (
    "/app/ros2_ws/install/oak_d_lite/lib/python3.12/site-packages/"
    "oak_d_lite/hand_tracking.py"
)
MODEL_ID = "hand_tracking_fast"
KEYPOINT_COUNT = 21
ROSBRIDGE_DEFAULT_TIMEOUT_SECONDS = 5.0
MODEL_CALL_TIMEOUT_SECONDS = 90.0
MIN_HAND_RATE_HZ = 10.0
MIN_MEASURE_SECONDS = 20.0

sys.path.insert(0, str(REPO_ROOT))
from ros_packages.camera.oak_d_lite.hand_tracking import (  # noqa: E402
    build_fast_tracker_script,
)
from ros_packages.camera.oak_d_lite.pipeline_manager import (  # noqa: E402
    MODEL_LIFECYCLE_CALL_TIMEOUT_SECONDS,
    ROSBRIDGE_DEFAULT_CALL_SERVICE_TIMEOUT_SECONDS,
)


def _function(tree: ast.AST, name: str) -> ast.FunctionDef:
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise SystemExit(f"missing function {name}")


def source_identity() -> dict:
    script = build_fast_tracker_script(256, 144)
    tree = ast.parse(script)
    tensor_values = _function(tree, "tensor_values")
    reads = [
        node.func.attr
        for node in ast.walk(tensor_values)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr.startswith(("get", "set", "has"))
    ]
    return {
        "path": str(HAND_TRACKING.relative_to(REPO_ROOT)),
        "tensor_reads": reads,
        "uses_getLayerFp16": "getLayerFp16" in reads,
        "uses_getTensor": "getTensor" in reads,
    }


def frame_path_inventory() -> dict:
    """Queues in build_fast_hand_graph. Crop config stays on the device."""
    tree = ast.parse(HAND_TRACKING.read_text(encoding="utf-8"))
    graph = _function(tree, "build_fast_hand_graph")
    host_output_queues = []
    host_input_queues = []
    links = []
    creates = []
    for node in ast.walk(graph):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            name = node.func.attr
            if name == "create":
                if node.args and isinstance(node.args[0], ast.Attribute):
                    creates.append(node.args[0].attr)
            elif name == "createOutputQueue":
                host_output_queues.append(ast.dump(node))
            elif name == "createInputQueue":
                host_input_queues.append(ast.dump(node))
            elif name == "link" and node.args:
                links.append(ast.dump(node.args[0]))
    return {
        "created_nodes": creates,
        "host_output_queues": len(host_output_queues),
        "host_input_queues": len(host_input_queues),
        "hostnode_present": "HostNode" in creates,
        "device_links": len(links),
        "host_queue": "script detections output, maxSize=1, blocking=False",
        "device_only": [
            "pre_pd_manip_cfg -> palm ImageManip inputConfig",
            "pre_lm_manip_cfg -> landmark ImageManip inputConfig",
            "from_post_pd_nn <- decoder output",
            "from_lm_nn <- landmark output",
            "256x144 camera branch -> both ImageManip inputImage links",
        ],
    }


def packaging_install(tmp_root: Path) -> dict:
    library = tmp_root / "site-packages"
    library.mkdir(parents=True)
    try:
        subprocess.check_call(
            [
                sys.executable,
                "setup.py",
                "install",
                "--single-version-externally-managed",
                "--root",
                str(tmp_root / "root"),
                "--prefix",
                "/usr",
                "--install-lib",
                str(library),
                "--record",
                str(tmp_root / "install-record.txt"),
            ],
            cwd=CAMERA_SETUP.parent,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    finally:
        for generated in (
            CAMERA_SETUP.parent / "build",
            CAMERA_SETUP.parent / "oak_d_lite.egg-info",
        ):
            if generated.exists():
                subprocess.check_call(["rm", "-rf", str(generated)])
    installed_files = list((tmp_root / "root").rglob("hand_tracking.py"))
    if len(installed_files) != 1:
        raise SystemExit(
            f"expected one installed hand_tracking.py, found {installed_files}"
        )
    installed = installed_files[0]
    text = installed.read_text(encoding="utf-8")
    return {
        "installed_relative": f"{installed.parent.name}/{installed.name}",
        "matches_source": text == HAND_TRACKING.read_text(encoding="utf-8"),
        "uses_getLayerFp16": "nn_data.getLayerFp16(name)" in text,
        "uses_getTensor": "nn_data.getTensor" in text,
        "container_module": INSTALLED_MODULE,
    }


def local_report() -> dict:
    import tempfile

    with tempfile.TemporaryDirectory() as tmp:
        packaging = packaging_install(Path(tmp))
    identity = source_identity()
    inventory = frame_path_inventory()
    problems = []
    if not identity["uses_getLayerFp16"] or identity["uses_getTensor"]:
        problems.append("source tensor read is not device-safe")
    if not packaging["matches_source"] or not packaging["uses_getLayerFp16"]:
        problems.append("setuptools install did not ship the device-safe module")
    if packaging["uses_getTensor"]:
        problems.append("installed module still calls getTensor")
    if inventory["hostnode_present"] or inventory["host_input_queues"] != 0:
        problems.append("host crop queue or HostNode is present")
    if inventory["host_output_queues"] != 1:
        problems.append("expected exactly one host output queue")
    if (
        MODEL_LIFECYCLE_CALL_TIMEOUT_SECONDS
        <= ROSBRIDGE_DEFAULT_CALL_SERVICE_TIMEOUT_SECONDS
    ):
        problems.append("model call timeout does not outlast rosbridge's default")
    return {
        "mode": "local",
        "live_acceptance": "NOT EXECUTED",
        "source": identity,
        "packaging": packaging,
        "frame_path": inventory,
        "rosbridge_default_timeout_seconds": ROSBRIDGE_DEFAULT_TIMEOUT_SECONDS,
        "model_call_timeout_seconds": MODEL_CALL_TIMEOUT_SECONDS,
        "problems": problems,
    }


def _ros_call(url: str, service: str, service_type: str, args: dict, timeout: float):
    import websocket

    connection = websocket.create_connection(url, timeout=timeout)
    try:
        request_id = f"hand-accept-{uuid.uuid4()}"
        connection.send(
            json.dumps(
                {
                    "op": "call_service",
                    "id": request_id,
                    "service": service,
                    "type": service_type,
                    "args": args,
                    "timeout": timeout,
                }
            )
        )
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            connection.settimeout(min(1.0, deadline - time.monotonic()))
            raw = connection.recv()
            if not raw:
                continue
            message = json.loads(raw)
            if message.get("id") == request_id:
                return message
        raise TimeoutError(f"no response from {service} within {timeout}s")
    finally:
        connection.close()


def live_report(args: argparse.Namespace) -> dict:
    """Hardware and browser steps. The caller must pass --execute-live."""
    if not args.rosbridge_url:
        raise SystemExit(
            "live mode needs PIB_ACCEPTANCE_ROSBRIDGE_URL or --rosbridge-url"
        )
    report = {
        "mode": "live",
        "live_acceptance": "EXECUTED",
        "rosbridge_url": args.rosbridge_url,
        "browser": "NOT EXECUTED",
    }
    if args.docker_container:
        inspected = subprocess.check_output(
            [
                "docker",
                "inspect",
                "--format",
                "{{.Image}} {{.Id}}",
                args.docker_container,
            ],
            text=True,
        ).strip()
        module = (
            subprocess.check_output(
                [
                    "docker",
                    "exec",
                    args.docker_container,
                    "python3",
                    "-c",
                    (
                        "import inspect, oak_d_lite.hand_tracking as module\n"
                        "print(module.__file__)\n"
                        "print('getLayerFp16' in inspect.getsource(module.tensor_values))\n"
                        "print('getTensor' in inspect.getsource(module.tensor_values))\n"
                    ),
                ],
                text=True,
            )
            .strip()
            .splitlines()
        )
        report["container"] = {
            "inspect": inspected,
            "module_file": module[0] if module else "",
            "tensor_uses_getLayerFp16": (
                module[1] == "True" if len(module) > 1 else False
            ),
            "tensor_uses_getTensor": module[2] == "True" if len(module) > 2 else True,
        }
    else:
        report["container"] = "NOT EXECUTED (pass --docker-container on the robot host)"

    started = _ros_call(
        args.rosbridge_url,
        "/start_model",
        "datatypes/srv/StartModel",
        {"model_id": MODEL_ID, "shaves": 0, "owner": args.owner},
        MODEL_CALL_TIMEOUT_SECONDS,
    )
    report["start"] = {
        "result": started.get("result"),
        "values": started.get("values"),
    }
    # Counting continues only after a truthful start. A rosbridge timeout
    # string is a failed start even if detections arrive later.
    if started.get("result") is not True or not isinstance(started.get("values"), dict):
        report["hand_rate"] = (
            "NOT EXECUTED (start was not a successful service response)"
        )
        return report
    if started["values"].get("success") is not True:
        report["hand_rate"] = "NOT EXECUTED (start reported failure)"
        return report

    import websocket

    connection = websocket.create_connection(args.rosbridge_url, timeout=5)
    try:
        connection.send(
            json.dumps(
                {
                    "op": "subscribe",
                    "id": f"hand-accept-sub-{uuid.uuid4()}",
                    "topic": f"/detections/{MODEL_ID}",
                    "type": "datatypes/msg/DetectionArray",
                }
            )
        )
        deadline = time.monotonic() + args.measure_seconds
        started_at = None
        total = 0
        with_hand = 0
        bad_shape = 0
        sample = None
        while time.monotonic() < deadline:
            connection.settimeout(min(1.0, deadline - time.monotonic()))
            try:
                raw = connection.recv()
            except Exception:
                continue
            if not raw:
                continue
            message = json.loads(raw)
            if message.get("topic") != f"/detections/{MODEL_ID}":
                continue
            body = message.get("msg") or {}
            if started_at is None:
                started_at = time.monotonic()
            total += 1
            detections = body.get("detections") or []
            hands = [
                item
                for item in detections
                if len(item.get("keypoint_names") or []) == KEYPOINT_COUNT
                and len(item.get("keypoint_x") or []) == KEYPOINT_COUNT
                and len(item.get("keypoint_y") or []) == KEYPOINT_COUNT
                and len(item.get("keypoint_z") or []) == KEYPOINT_COUNT
            ]
            if hands:
                with_hand += 1
                if sample is None:
                    sample = {
                        "model_id": body.get("model_id"),
                        "frame_width": body.get("frame_width"),
                        "frame_height": body.get("frame_height"),
                        "score": hands[0].get("score"),
                        "keypoint_names": hands[0].get("keypoint_names"),
                    }
            elif detections:
                bad_shape += 1
        elapsed = 0.0 if started_at is None else time.monotonic() - started_at
        rate = (with_hand / elapsed) if elapsed > 0 else 0.0
        report["hand_rate"] = {
            "total_messages": total,
            "hand_messages": with_hand,
            "bad_shape": bad_shape,
            "elapsed_seconds": round(elapsed, 3),
            "hand_rate_hz": round(rate, 3),
            "sample": sample,
            "meets_rate": rate >= MIN_HAND_RATE_HZ and elapsed >= MIN_MEASURE_SECONDS,
        }
    finally:
        connection.close()
    stopped = _ros_call(
        args.rosbridge_url,
        "/stop_model",
        "datatypes/srv/StopModel",
        {"model_id": MODEL_ID, "owner": args.owner},
        MODEL_CALL_TIMEOUT_SECONDS,
    )
    report["stop"] = {"result": stopped.get("result"), "values": stopped.get("values")}
    report["browser"] = (
        "NOT EXECUTED here. Run the Cerebra script "
        "scripts/verify-hand-overlay-acceptance.mjs against the production page "
        "during the same hand interval. ROS rate is not overlay FPS."
    )
    return report


def remaining_steps() -> list[str]:
    return [
        "Merge the reviewed backend and Cerebra branches. Do not treat unit tests as acceptance.",
        "Deploy with the normal update path so ros-camera is built from the merged tree, not from a copied file.",
        "Record the image id and the imported module path inside the running container. It must be "
        + INSTALLED_MODULE
        + " and tensor_values must call getLayerFp16.",
        "Force-recreate ros-camera with the normal compose path and repeat the module-path check. A recreated container must still import getLayerFp16.",
        "From Cerebra, start hand_tracking_fast. The rosbridge call must include timeout "
        f"{MODEL_CALL_TIMEOUT_SECONDS:g} (the server default is {ROSBRIDGE_DEFAULT_TIMEOUT_SECONDS:g}s). "
        "The response result must be true and values.success true before any detection is counted.",
        f"Hold one real hand in view for at least {MIN_MEASURE_SECONDS:g}s after warm-up. "
        f"Record total messages, hand-containing messages, interval, and inter-message rate. "
        f"Require at least {MIN_HAND_RATE_HZ:g} hand messages per second, each with {KEYPOINT_COUNT} named x/y/z keypoints and a positive frame size.",
        "During that same interval, on the production camera page, record distinct applied overlay updates and rendered keypoint positions. A static screenshot or the ROS rate alone is not UI evidence.",
        "Move the hand across the frame and resize the browser. Markers stay on the displayed hand. An empty result and model stop both clear the markers.",
        "Repeat start/stop. Logs must show no getTensor exception, no model-start fallback, no device crash, and no device-in-use loop.",
        "Confirm the frame path still has no HostNode and one host queue (detections, maxSize=1, blocking=False). SHAVE budget remains 4+1+4.",
    ]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--execute-live",
        action="store_true",
        help="Contact rosbridge and optional local docker. Omit this for NOT EXECUTED.",
    )
    parser.add_argument(
        "--rosbridge-url", default=os.environ.get("PIB_ACCEPTANCE_ROSBRIDGE_URL", "")
    )
    parser.add_argument(
        "--docker-container",
        default=os.environ.get("PIB_ACCEPTANCE_CAMERA_CONTAINER", ""),
    )
    parser.add_argument("--owner", default="pr1970-acceptance")
    parser.add_argument("--measure-seconds", type=float, default=MIN_MEASURE_SECONDS)
    args = parser.parse_args()

    report = local_report()
    report["remaining_post_merge_steps"] = remaining_steps()
    if args.execute_live:
        report["live"] = live_report(args)
    else:
        report["live"] = "NOT EXECUTED"
    print(json.dumps(report, indent=2))
    if report["problems"]:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
