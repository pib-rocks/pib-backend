"""German one-line descriptions for every MCP tool named in capability manifests.

This is the only place to edit tool prose for the generated SOUL section.
"""

from __future__ import annotations

MCP_TOOL_DESCRIPTIONS: dict[str, str] = {
    "list_motors": (
        "Listet konfigurierte Motoren und Bricklets inklusive aktueller Motorpositionen."
    ),
    "get_state": (
        "Liefert den aktuellen Gelenkzustand, Diagnosen und Roboter-Telemetrie."
    ),
    "get_motion_model": (
        "Liefert das Bewegungsmodell: Gelenke, Achsen, Grenzen in Grad und benannte "
        "Stellungen; nutze es, statt eine URDF zu verlangen."
    ),
    "list_poses": "Listet gespeicherte Posen.",
    "list_programs": "Listet gespeicherte Blockly-/Python-Programme.",
    "capture_image": "Nimmt ein Kamerabild als base64-kodiertes JPEG auf.",
    "move_motor": (
        "Bewegt einen Motor in Grad innerhalb seiner konfigurierten Rotationsgrenzen."
    ),
    "move_motors": (
        "Bewegt mehrere Motoren in einem Aufruf (Zielwinkel in Grad pro Motor)."
    ),
    "apply_joint_trajectory": (
        "Führt eine zeitgesteuerte Gelenk-Trajektorie aus (Grad an der Tool-Grenze, "
        "Radiant auf ROS)."
    ),
    "apply_pose": "Wendet eine gespeicherte Pose anhand ihres genauen Namens an.",
    "run_program": "Startet ein gespeichertes Programm anhand seiner Program-ID.",
    "set_led": "Setzt die RGB-LED eines Buttons (Button 1–3, Kanäle 0–255).",
    "set_relay": "Schaltet das Solid-State-Relais ein oder aus.",
    "list_smarthome_devices": (
        "Listet SmartHome-Geräte für die Sprachsteuerung; optional gefiltert nach "
        "Raum oder Domäne."
    ),
    "get_smarthome_state": (
        "Liefert den Zustand eines SmartHome-Geräts (Entitäts-ID oder Anzeigename)."
    ),
    "control_smarthome_device": (
        "Führt eine SmartHome-Aktion an einem Gerät aus; optional mit numerischem "
        "oder Text-Wert."
    ),
    "trigger_smarthome_scene": ("Aktiviert eine SmartHome-Szene anhand ihres Namens."),
    "soul_append": (
        "Hängt eine dauerhafte Lektion an die SOUL.md einer Persönlichkeit an; "
        "ersetzt sie nie."
    ),
    "pib_recall": (
        "Ruft Episodenerinnerungen ab (read-only, top-k, mit Datum und Belegen)."
    ),
    "pib_remember": (
        "Speichert eine Episodenerinnerung nur, wenn der Nutzer ausdrücklich darum "
        "bittet; zum Abrufen nutze pib_recall."
    ),
    "pib_describe_scene": (
        "Liefert eine Zusammenfassung der aktuellen Kamera-Szene (Objekte, Personen, "
        "Entfernungen in Metern)."
    ),
    "pib_locate_object": (
        "Findet ein beschriftetes Objekt in der Szene (Entfernung in Metern); "
        "Groß-/Kleinschreibung beim Label ist egal."
    ),
    "pib_web_search": (
        "Recherchiert aktuelle Informationen aus dem Internet (Wetter, Fahrpläne, "
        "Veranstaltungen, Nachrichten, Fakten) über pib.Cloud."
    ),
}


def description_for_tool(tool: str) -> str:
    """Return the German sentence for one manifest skill name."""
    try:
        return MCP_TOOL_DESCRIPTIONS[tool]
    except KeyError as error:
        raise KeyError(f"no MCP tool description for {tool!r}") from error


def skills_missing_descriptions(skills: list[str]) -> list[str]:
    """Skill names from a manifest that have no entry in ``MCP_TOOL_DESCRIPTIONS``."""
    return [name for name in skills if name not in MCP_TOOL_DESCRIPTIONS]


def description_keys_not_in_skills(skills: frozenset[str]) -> list[str]:
    """Description keys that do not appear in the given skill-name set."""
    return sorted(key for key in MCP_TOOL_DESCRIPTIONS if key not in skills)


def union_manifest_skill_names() -> frozenset[str]:
    """Every skill listed in any bundled capability manifest."""
    from pib_capability.manifest import AVAILABLE_VARIANTS, load_manifest

    names: set[str] = set()
    for variant in AVAILABLE_VARIANTS:
        names.update(load_manifest(variant)["skills"])
    return frozenset(names)


def validate_mcp_tool_descriptions() -> None:
    """Every manifest skill has prose; every prose key names a manifest skill."""
    skills = union_manifest_skill_names()
    missing = skills_missing_descriptions(sorted(skills))
    if missing:
        raise ValueError(
            "manifest skills missing MCP tool descriptions: "
            + ", ".join(sorted(missing))
        )
    orphan = description_keys_not_in_skills(skills)
    if orphan:
        raise ValueError(
            "MCP tool descriptions not listed in any manifest: " + ", ".join(orphan)
        )
