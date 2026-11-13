"""Merge a partial ``ApplyMotorSettings`` request onto a motor's current settings (PR-2319).

A ``MotorSettings`` message cannot express "field not provided": every numeric field
that the caller leaves out arrives as ``0`` and every boolean as ``False``. Applying
such a document writes zeros to the motor (pulse widths, rotation ranges, acceleration,
deceleration), disables it (``turned_on=False``) and wipes ``visible`` - which is what
a documented ``Write.set("motor", velocity=...)`` call did before this module existed.

``ApplyMotorSettings.apply_fields`` lets a caller name the fields it actually wants to
change. This module turns that into the complete document which is then applied, so
every other field keeps the value the motor has right now.

The module is deliberately free of ROS imports so it can be unit-tested without a
running node.
"""

from typing import Any, Mapping, Sequence

# Message field name (snake_case, as in MotorSettings.msg) -> DTO key used by the
# actuator layer. ``motor_name`` is carried separately: it selects the motor and is
# never part of the settings document itself.
FIELD_TO_DTO: dict[str, str] = {
    "turned_on": "turnedOn",
    "pulse_width_min": "pulseWidthMin",
    "pulse_width_max": "pulseWidthMax",
    "rotation_range_min": "rotationRangeMin",
    "rotation_range_max": "rotationRangeMax",
    "velocity": "velocity",
    "acceleration": "acceleration",
    "deceleration": "deceleration",
    "period": "period",
    "visible": "visible",
    "upper_endpoint": "upperEndpoint",
}

APPLYABLE_FIELDS: tuple[str, ...] = tuple(FIELD_TO_DTO)


class UnknownApplyFields(ValueError):
    """Raised when a request names fields the message does not have."""

    def __init__(self, fields: Sequence[str]):
        self.fields = sorted(set(fields))
        super().__init__(
            "unknown apply_fields: "
            + ", ".join(self.fields)
            + " (known: "
            + ", ".join(APPLYABLE_FIELDS)
            + ")"
        )


def unknown_apply_fields(apply_fields: Sequence[str] | None) -> list[str]:
    """Return the names in ``apply_fields`` that are not settings fields."""
    return sorted(
        {field for field in (apply_fields or []) if field not in FIELD_TO_DTO}
    )


def merge_partial_settings(
    current: Mapping[str, Any],
    incoming: Mapping[str, Any],
    apply_fields: Sequence[str] | None,
) -> dict[str, Any]:
    """Return the settings document to apply.

    ``current`` is the motor's present settings (``Motor.get_settings()``), ``incoming``
    is the DTO built from the request message. When ``apply_fields`` is empty the
    incoming document is applied as-is - the historical behaviour, kept so existing
    callers that send a complete document keep working. Otherwise only the named
    fields are taken from the request and every other field keeps its current value.

    Raises:
        UnknownApplyFields: a name in ``apply_fields`` is not a settings field.
    """
    unknown = unknown_apply_fields(apply_fields)
    if unknown:
        raise UnknownApplyFields(unknown)

    fields = [field for field in (apply_fields or []) if field]
    if not fields:
        return dict(incoming)

    merged = dict(current)
    for field in fields:
        dto_key = FIELD_TO_DTO[field]
        merged[dto_key] = incoming[dto_key]
    if "name" in current:
        merged["name"] = current["name"]
    return merged


def copy_dto_into_motor_settings(dto: Mapping[str, Any], motor_settings: Any) -> None:
    """Write a DTO back into a ``MotorSettings`` message so the echo is truthful.

    The service publishes the applied settings on ``/motor_settings``; publishing the
    raw request would announce the zeros the caller did not send.
    """
    for message_field, dto_key in FIELD_TO_DTO.items():
        if dto_key in dto:
            setattr(motor_settings, message_field, dto[dto_key])
