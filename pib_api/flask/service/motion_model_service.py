"""Kinematic self-description for Direct LLM turns, sourced from the motor DB."""

from __future__ import annotations

from typing import Any

from app.app import db
from model.motor_model import Motor
from model.pose_model import Pose

UNITS = {
    "angle": "centidegree",
    "note": "1 = 0.01 degree; 0 = the joint zero; +/-9000 = +/-90 degrees",
}

_UNITS_LINE = (
    "Body (centidegrees: 1 = 0.01 degree, 0 = joint zero, " "+/-9000 = +/-90 degrees)."
)

_CHAIN_ORDER = (
    "head",
    "left_arm",
    "right_arm",
    "left_hand",
    "right_hand",
    "other",
)

# Cap posture names in the model; only the first three carry per-motor positions in
# the prompt text to keep Direct turns small.
_MAX_POSTURES = 10
_MAX_POSTURE_DETAIL = 3


def _chain_templates() -> dict[str, list[str]]:
    templates: dict[str, list[str]] = {
        "head": ["turn_head_motor", "tilt_forward_motor"],
    }
    for side in ("left", "right"):
        templates[f"{side}_arm"] = [
            f"shoulder_vertical_{side}",
            f"shoulder_horizontal_{side}",
            f"upper_arm_{side}_rotation",
            f"elbow_{side}",
            f"lower_arm_{side}_rotation",
            f"wrist_{side}",
        ]
        templates[f"{side}_hand"] = [
            f"thumb_{side}_opposition",
            f"thumb_{side}_stretch",
            f"index_{side}_stretch",
            f"middle_{side}_stretch",
            f"ring_{side}_stretch",
            f"pinky_{side}_stretch",
        ]
    return templates


def _joint_from_motor(motor: Motor) -> dict[str, Any]:
    lower = motor.rotation_range_min
    upper = motor.rotation_range_max
    if motor.upper_endpoint == "max":
        upper_end_value = upper
        lower_end_value = lower
    else:
        upper_end_value = lower
        lower_end_value = upper
    return {
        "motor": motor.name,
        "min": lower,
        "max": upper,
        "upperEndpoint": motor.upper_endpoint,
        "upperEndValue": upper_end_value,
        "lowerEndValue": lower_end_value,
        "visible": motor.visible,
    }


def _chain_label(chain_key: str) -> str:
    if chain_key == "head":
        return "Head"
    if chain_key == "other":
        return "Other"
    return chain_key.replace("_", " ").title()


def _render_motor(joint: dict[str, Any]) -> str:
    text = (
        f"{joint['motor']} {joint['min']}..{joint['max']} "
        f"(upper end {joint['upperEndValue']})"
    )
    if not joint["visible"]:
        text += " (hidden from the UI)"
    return text


def build_motion_model() -> dict[str, Any]:
    motors = Motor.query.order_by(Motor.id).all()
    by_name = {motor.name: motor for motor in motors}
    templates = _chain_templates()
    assigned: set[str] = set()
    chains: dict[str, dict[str, Any]] = {}

    for chain_key in _CHAIN_ORDER:
        if chain_key == "other":
            continue
        joints: list[dict[str, Any]] = []
        for motor_name in templates[chain_key]:
            motor = by_name.get(motor_name)
            if motor is None:
                continue
            joints.append(_joint_from_motor(motor))
            assigned.add(motor_name)
        if joints:
            chains[chain_key] = {"joints": joints}

    other_joints = [
        _joint_from_motor(motor) for motor in motors if motor.name not in assigned
    ]
    if other_joints:
        chains["other"] = {"joints": other_joints}

    poses = Pose.query.order_by(Pose.name).limit(_MAX_POSTURES).all()
    postures: dict[str, dict[str, int]] = {}
    for pose in poses:
        postures[pose.name] = {
            position.motor_name: position.position for position in pose.motor_positions
        }

    return {
        "schemaVersion": 1,
        "units": UNITS,
        "chains": chains,
        "postures": postures,
    }


def render_prompt(model: dict[str, Any]) -> str:
    lines = [_UNITS_LINE]
    chains = model.get("chains") or {}
    for chain_key in _CHAIN_ORDER:
        chain = chains.get(chain_key)
        if not chain:
            continue
        joints = chain.get("joints") or []
        if not joints:
            continue
        motors_text = ", ".join(_render_motor(joint) for joint in joints)
        lines.append(f"{_chain_label(chain_key)}: {motors_text}.")

    postures = model.get("postures") or {}
    if postures:
        names = list(postures.keys())
        lines.append(f"Postures you can apply: {', '.join(names)}.")
        for name in names[:_MAX_POSTURE_DETAIL]:
            positions = postures[name]
            if not positions:
                continue
            parts = ", ".join(
                f"{motor}={value}" for motor, value in sorted(positions.items())
            )
            lines.append(f"  {name}: {parts}")

    if len(lines) == 1 and not chains:
        return lines[0]
    return "\n".join(lines)


def get_motion_model() -> dict[str, Any]:
    model = build_motion_model()
    return {**model, "prompt": render_prompt(model)}
