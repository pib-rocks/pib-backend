"""Registry-backed facts about this robot.

This table is intentionally limited to non-sensitive system facts. Never store
tokens, passwords, credentials, or other secrets in system properties.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import logging
from typing import Any, Callable

from sqlalchemy.dialects.sqlite import insert as sqlite_insert

from app.app import db
from model.system_property_model import SystemProperty

logger = logging.getLogger(__name__)

HARDWARE_VARIANT_KEY = "hardware.variant"
SOFTWARE_VERSION_KEY = "software.version"
MICROPHONE_DESIRED_STATE_KEY = "microphone.desired_state"
DEFAULT_HARDWARE_VARIANT = "pib5edu"
ALLOWED_HARDWARE_VARIANTS = (
    "pib4edu",
    "pib4advanced",
    "pib5advanced",
    "pib5museum",
    "pib5edu",
)
PROPERTY_SOURCES = frozenset({"default", "file", "environment", "migration", "command"})


@dataclass(frozen=True)
class PropertyDefinition:
    value_type: type
    validator: Callable[[Any], bool]


PROPERTY_REGISTRY = {
    HARDWARE_VARIANT_KEY: PropertyDefinition(
        str, lambda value: value in ALLOWED_HARDWARE_VARIANTS
    ),
    SOFTWARE_VERSION_KEY: PropertyDefinition(str, lambda value: bool(value.strip())),
    MICROPHONE_DESIRED_STATE_KEY: PropertyDefinition(dict, lambda value: bool(value)),
}


def _definition_for(key: str) -> PropertyDefinition:
    try:
        return PROPERTY_REGISTRY[key]
    except KeyError as error:
        raise ValueError(f"Unknown system property key: {key!r}") from error


def _validate(key: str, value: Any) -> PropertyDefinition:
    definition = _definition_for(key)
    if type(value) is not definition.value_type:
        raise ValueError(
            f"System property {key!r} requires "
            f"{definition.value_type.__name__}, got {type(value).__name__}"
        )
    if not definition.validator(value):
        raise ValueError(f"Invalid value for system property {key!r}: {value!r}")
    return definition


def get_property(key: str) -> SystemProperty | None:
    _definition_for(key)
    return db.session.get(SystemProperty, key)


def list_properties() -> list[SystemProperty]:
    return (
        SystemProperty.query.filter(SystemProperty.key.in_(PROPERTY_REGISTRY))
        .order_by(SystemProperty.key)
        .all()
    )


def _serialized_value(definition: PropertyDefinition, value: Any) -> str:
    if definition.value_type is dict:
        return json.dumps(value, separators=(",", ":"), sort_keys=True)
    return value


def get_property_value(key: str) -> Any | None:
    system_property = get_property(key)
    if system_property is None:
        return None

    definition = _definition_for(key)
    value = (
        json.loads(system_property.value)
        if definition.value_type is dict
        else system_property.value
    )
    _validate(key, value)
    return value


def set_property(key: str, value: Any, source: str) -> SystemProperty:
    definition = _validate(key, value)
    if source not in PROPERTY_SOURCES:
        raise ValueError(f"Unknown system property source: {source!r}")

    system_property = db.session.get(SystemProperty, key)
    if system_property is None:
        system_property = SystemProperty(key=key)
        db.session.add(system_property)

    system_property.value = _serialized_value(definition, value)
    system_property.value_type = definition.value_type.__name__
    system_property.source = source
    db.session.flush()
    return system_property


def set_property_if_missing(key: str, value: Any, source: str) -> SystemProperty:
    """Insert a property atomically without changing an existing value."""

    definition = _validate(key, value)
    if source not in PROPERTY_SOURCES:
        raise ValueError(f"Unknown system property source: {source!r}")

    statement = (
        sqlite_insert(SystemProperty)
        .values(
            key=key,
            value=_serialized_value(definition, value),
            value_type=definition.value_type.__name__,
            source=source,
        )
        .on_conflict_do_nothing(index_elements=[SystemProperty.key])
    )
    db.session.execute(statement)
    db.session.flush()
    return db.session.get(SystemProperty, key)


def get_variant() -> str:
    system_property = get_property(HARDWARE_VARIANT_KEY)
    if system_property is None:
        logger.warning(
            "No stored hardware variant; falling back to %s",
            DEFAULT_HARDWARE_VARIANT,
        )
        return DEFAULT_HARDWARE_VARIANT
    if system_property.value not in ALLOWED_HARDWARE_VARIANTS:
        logger.warning(
            "Unknown stored hardware variant %r; falling back to %s",
            system_property.value,
            DEFAULT_HARDWARE_VARIANT,
        )
        return DEFAULT_HARDWARE_VARIANT
    return system_property.value
