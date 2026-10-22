"""Compact robot motion model bundled for the MCP tools.

The JSON is generated from pib-sdk (``pib_sdk.motion_model``) and checked in, so
the voice-assistant container can answer questions about joints, axes, limits and
named postures without installing pinocchio, numpy or roslibpy. Regenerate it with
``python tools/build_mcp_motion_model.py`` after a pib-sdk release; ``--check``
fails when the file drifted.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

MOTION_MODEL_PATH = Path(__file__).resolve().parent / "data" / "motion_model.json"


def load_motion_model() -> dict[str, Any]:
    """Load the bundled motion model."""
    return json.loads(MOTION_MODEL_PATH.read_text(encoding="utf-8"))


def motion_model_text(model: dict[str, Any]) -> str:
    """Render the model exactly like ``pib_sdk.motion_model.prompt_text``.

    Limits are degrees, the axes are unit vectors in the URDF frame. The format is
    kept byte-identical to the SDK rendering so the drift test can compare both.
    """
    lines = [
        "pib motion model (angles deg, positions mm in kinematics API)",
        f"schema {model['schema_version']} from {model['source']['urdf_file']}",
    ]
    for chain_name in sorted(model["chains"]):
        chain = model["chains"][chain_name]
        lines.append(f"{chain_name} tip={chain['tip_frame']}")
        for joint in chain["joints"]:
            axis = joint["axis"]
            lines.append(
                f"  {joint['motor']}: "
                f"[{joint['lower_deg']}, {joint['upper_deg']}] "
                f"axis=({axis[0]}, {axis[1]}, {axis[2]}) "
                f"urdf={joint['urdf_joint']}"
            )
        if chain["named_configurations"]:
            parts = []
            for name in sorted(chain["named_configurations"]):
                values = chain["named_configurations"][name]
                formatted = ",".join(str(value) for value in values)
                parts.append(f"{name}=({formatted})")
            lines.append(f"  named: {'; '.join(parts)}")
    return "\n".join(lines) + "\n"


def joint_table(model: dict[str, Any]) -> list[dict[str, Any]]:
    """Flat list of the commandable joints, one row per motor."""
    rows: list[dict[str, Any]] = []
    for chain_name in sorted(model["chains"]):
        chain = model["chains"][chain_name]
        for joint in chain["joints"]:
            rows.append(
                {
                    "chain": chain_name,
                    "tipFrame": chain["tip_frame"],
                    "motor": joint["motor"],
                    "urdfJoint": joint["urdf_joint"],
                    "type": joint["type"],
                    "axis": list(joint["axis"]),
                    "lowerDeg": joint["lower_deg"],
                    "upperDeg": joint["upper_deg"],
                }
            )
    return rows


def motion_model_payload() -> dict[str, Any]:
    """What the MCP tool returns: prompt text, structured joints and provenance."""
    model = load_motion_model()
    return {
        "text": motion_model_text(model),
        "joints": joint_table(model),
        "namedConfigurations": {
            chain_name: model["chains"][chain_name]["named_configurations"]
            for chain_name in sorted(model["chains"])
        },
        "source": model["source"],
    }
