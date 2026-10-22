"""The bundled motion model: provenance, size budget and the MCP surface.

The artifact is generated from pib-sdk by ``tools/build_mcp_motion_model.py``.
The unit suite pins the table below so an accidental edit of the JSON fails here;
upstream drift (a new pib-sdk release changing a URDF limit) is caught by
``python tools/build_mcp_motion_model.py --check`` with pib-sdk installed, which is
why the artifact records the SDK version and the URDF hash it came from.
"""

from __future__ import annotations

import asyncio
import json

from pib_mcp_server.backend import PibBackend
from pib_mcp_server.motion_model import (
    MOTION_MODEL_PATH,
    joint_table,
    load_motion_model,
    motion_model_payload,
    motion_model_text,
)
from pib_mcp_server.server import create_server

MAX_JSON_BYTES = 8_192
MAX_TEXT_BYTES = 4_096

#: (chain, motor, lower_deg, upper_deg) exactly as pib-sdk 0.5.6 generated them.
EXPECTED_JOINTS = (
    ("head", "turn_head_motor", -90.0, 90.0),
    ("head", "tilt_forward_motor", -45.0, 70.0),
    ("left_arm", "shoulder_vertical_left", -90.0, 90.0),
    ("left_arm", "shoulder_horizontal_left", -90.0, 90.0),
    ("left_arm", "upper_arm_left_rotation", -90.0, 90.0),
    ("left_arm", "elbow_left", -45.0, 90.0),
    ("left_arm", "lower_arm_left_rotation", -90.0, 90.0),
    ("left_arm", "wrist_left", -30.0, 30.0),
    ("right_arm", "shoulder_vertical_right", -90.0, 90.0),
    ("right_arm", "shoulder_horizontal_right", -90.0, 90.0),
    ("right_arm", "upper_arm_right_rotation", -90.0, 90.0),
    ("right_arm", "elbow_right", -45.0, 90.0),
    ("right_arm", "lower_arm_right_rotation", -90.0, 90.0),
    ("right_arm", "wrist_right", -30.0, 30.0),
)


def _call(server, name: str, arguments: dict) -> dict:
    result = asyncio.run(server.call_tool(name, arguments))
    if isinstance(result, tuple):
        return result[1]
    content = getattr(result, "content", None)
    if content:
        return json.loads(content[0].text)
    return result


def test_artifact_fits_the_prompt_budget():
    raw = MOTION_MODEL_PATH.read_bytes()
    text = motion_model_text(load_motion_model()).encode("utf-8")
    assert len(raw) <= MAX_JSON_BYTES, f"artifact is {len(raw)} bytes"
    assert len(text) <= MAX_TEXT_BYTES, f"prompt text is {len(text)} bytes"


def test_every_motor_is_listed_with_its_limits_and_axis():
    joints = joint_table(load_motion_model())
    actual = tuple(
        (row["chain"], row["motor"], row["lowerDeg"], row["upperDeg"]) for row in joints
    )
    assert actual == EXPECTED_JOINTS
    text = motion_model_text(load_motion_model())
    for chain, motor, lower, upper in EXPECTED_JOINTS:
        assert f"{motor}: [{lower}, {upper}]" in text
        assert f"{chain} tip=" in text
    for row in joints:
        assert len(row["axis"]) == 3
        assert row["urdfJoint"]


def test_artifact_records_where_it_came_from():
    source = load_motion_model()["source"]
    assert source["urdf_file"] == "pib_model.urdf"
    assert source["sdk_version"] == "0.5.6"
    assert source["urdf_sha256"] == (
        "a35af35f41641040e181a2b43acd4dd4d8e3c63d30c3e925c7a0ffb0e3a9b433"
    )


def test_named_postures_are_served_for_every_arm():
    payload = motion_model_payload()
    assert set(payload["namedConfigurations"]["right_arm"]) >= {
        "zero",
        "observe",
        "rest",
    }
    assert set(payload["namedConfigurations"]["head"]) >= {"zero"}


def test_mcp_tool_returns_the_model_without_actuation():
    server = create_server(PibBackend(), actuating_enabled=False)
    payload = _call(server, "get_motion_model", {})
    assert payload["ok"] is True
    model = payload["result"]
    assert len(model["joints"]) == len(EXPECTED_JOINTS)
    assert model["text"].startswith("pib motion model")
    assert model["source"]["urdf_file"] == "pib_model.urdf"


def test_soul_section_advertises_the_tool_with_its_exact_name():
    from pib_hermes_config import MCP_TOOLS, MCP_TOOLS_SOUL_SECTION, mcp_tool_name

    assert "get_motion_model" in {tool for tool, _ in MCP_TOOLS}
    assert f"### {mcp_tool_name('get_motion_model')}" in MCP_TOOLS_SOUL_SECTION
