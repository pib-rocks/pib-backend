"""Flask CLI commands.

Docker Compose runs ``seed_db`` when the API container starts. A selected hardware
variant without an implemented profile therefore fails startup loudly instead of
silently seeding hardware for a different robot.
"""

import logging
import sqlite3
from datetime import datetime, timezone
from pathlib import Path

import click
from sqlalchemy import inspect
from sqlalchemy.engine import URL, make_url

from app.app import db, app
from model.assistant_model import AssistantModel
from model.controller_model import Controller
from model.camera_settings_model import CameraSettings
from model.chat_message_model import ChatMessage
from model.chat_model import Chat
from model.motor_model import Motor
from model.personality_model import Personality
from model.program_model import Program
from model.pose_model import Pose
from model.motor_position_model import MotorPosition
from seed_profiles import (
    HardwareProfile,
    UnknownHardwareVariantError,
    get_profile,
    resolve_variant_and_source,
)
from service.system_property_service import (
    ALLOWED_HARDWARE_VARIANTS,
    HARDWARE_VARIANT_KEY,
    SOFTWARE_VERSION_KEY,
    get_property,
    set_property,
)
from service.version_service import read_app_version
from default_pose_constants import (
    STARTUP_POSITIONS,
    CALIBRATION_POSITIONS,
    STARTUP_POSE_NAME,
    CALIBRATION_POSE_NAME,
)
from model.button_program_model import ButtonProgram
from service.microphone_array_service import seed_desired_state

logger = logging.getLogger(__name__)


@app.cli.command("seed_db")
def seed_db() -> None:
    if not _is_empty_db():
        print("Seeding database failed - database already contains data.")
        return
    variant, source = resolve_variant_and_source()
    profile = get_profile(variant)
    print(
        f"Seeding hardware variant {variant!r} using profile "
        f"{profile.description!r}."
    )
    _create_controller_data(profile)
    _create_camera_data()
    _create_program_data()
    _create_chat_data_and_assistant()
    _create_default_poses(profile)
    _create_button_program_data(profile)
    set_property(HARDWARE_VARIANT_KEY, variant, source)
    seed_desired_state(profile)
    db.session.commit()
    print("Seeded the database with default data.")


@app.cli.command("seed_hardware")
@click.option(
    "--variant",
    required=True,
    type=click.Choice(ALLOWED_HARDWARE_VARIANTS, case_sensitive=True),
)
@click.option("--force", is_flag=True)
def seed_hardware(variant: str, force: bool) -> None:
    """Deliberately replace the hardware layout on an existing machine."""
    try:
        profile = get_profile(variant)
    except UnknownHardwareVariantError as error:
        raise click.ClickException(str(error)) from error

    if not force:
        raise click.ClickException(
            "Refusing to rebuild hardware without --force. No changes were made."
        )

    # refuse a database that cannot be backed up before asking the operator to confirm
    _file_backed_sqlite_url()

    try:
        confirmation = input(
            f"Type the hardware variant name {variant!r} exactly to continue: "
        )
    except EOFError:
        confirmation = ""
    if confirmation != variant:
        raise click.ClickException("Confirmation did not match. No changes were made.")

    backup_path = _backup_sqlite_database()
    click.echo(f"Database backup: {backup_path}")

    protected_before = _protected_counts()
    try:
        controller_stats = _upsert_controllers(profile)
        motor_stats, warnings = _upsert_motors(profile)
        _rebuild_button_programs(profile)
        _delete_obsolete_controllers(profile, controller_stats)
        set_property(HARDWARE_VARIANT_KEY, variant, "command")

        protected_after = _protected_counts()
        orphaned_motor_names = _orphaned_motor_position_names()
        if orphaned_motor_names:
            warnings.append(
                "motor_position rows reference missing motors: "
                + ", ".join(orphaned_motor_names)
            )
        db.session.commit()
    except Exception:
        db.session.rollback()
        raise

    for warning in warnings:
        click.echo(f"WARNING: {warning}")
    click.echo(f"Hardware variant: {variant}")
    click.echo(f"Profile: {profile.description}")
    click.echo(
        "Controllers: "
        f"{controller_stats['created']} created, "
        f"{controller_stats['updated']} updated, "
        f"{controller_stats['deleted']} deleted"
    )
    click.echo(
        "Motors: "
        f"{motor_stats['created']} created, "
        f"{motor_stats['updated']} updated, "
        f"{motor_stats['deleted']} deleted"
    )
    click.echo(f"Backup: {backup_path}")
    click.echo(
        "Protected counts unchanged: "
        + ", ".join(
            f"{name}={'yes' if protected_before[name] == protected_after[name] else 'NO'}"
            for name in protected_before
        )
    )


def _file_backed_sqlite_url(database_url: URL | None = None) -> URL:
    """Return the URL of the SQLite database this command will modify.

    The path is taken from the engine that is actually in use instead of the configured URL: the two
    can diverge and the backup has to copy the database that is really about to change.
    """
    url = make_url(str(db.engine.url)) if database_url is None else database_url
    if url.get_backend_name() != "sqlite":
        raise click.ClickException(
            "seed_hardware requires a file-backed SQLite database; "
            f"configured URL uses {url.get_backend_name()!r}."
        )
    if not url.database or url.database == ":memory:":
        raise click.ClickException(
            "seed_hardware requires a file-backed SQLite database; "
            "in-memory SQLite cannot be backed up."
        )
    return url


def _backup_sqlite_database(database_url: URL | None = None) -> Path:
    database_url = _file_backed_sqlite_url(database_url)

    database_path = Path(database_url.database).expanduser().resolve()
    if not database_path.is_file():
        raise click.ClickException(
            f"SQLite database file does not exist: {database_path}"
        )

    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    backup_path = Path(f"{database_path}.bak-{timestamp}")
    try:
        with (
            sqlite3.connect(database_path) as source,
            sqlite3.connect(backup_path) as destination,
        ):
            source.backup(destination)
    except (OSError, sqlite3.Error) as error:
        backup_path.unlink(missing_ok=True)
        raise click.ClickException(
            f"Could not back up SQLite database: {error}"
        ) from error
    return backup_path


def _protected_counts() -> dict[str, int]:
    return {
        "pose": Pose.query.count(),
        "program": Program.query.count(),
        "chat": Chat.query.count(),
    }


def _upsert_controllers(profile: HardwareProfile) -> dict[str, int]:
    stats = {"created": 0, "updated": 0, "deleted": 0}
    for entry in profile.controllers:
        controller = Controller.query.filter_by(number=entry.number).one_or_none()
        if controller is None:
            controller = Controller(
                number=entry.number,
                kind=entry.kind,
                device_type=entry.device_type,
                supply_voltage=entry.supply_voltage,
                address=entry.address,
            )
            db.session.add(controller)
            stats["created"] += 1
        else:
            controller.kind = entry.kind
            controller.device_type = entry.device_type
            controller.supply_voltage = entry.supply_voltage
            stats["updated"] += 1
    db.session.flush()
    return stats


def _upsert_motors(
    profile: HardwareProfile,
) -> tuple[dict[str, int], list[str]]:
    stats = {"created": 0, "updated": 0, "deleted": 0}
    warnings: list[str] = []
    controllers = {
        controller.number: controller
        for controller in Controller.query.filter(
            Controller.number.in_(
                {number for number, _channel in profile.motor_mapping.values()}
            )
        ).all()
    }

    for motor_name, (controller_number, channel) in profile.motor_mapping.items():
        motor = Motor.query.filter_by(name=motor_name).one_or_none()
        if motor is None:
            settings = dict(profile.motor_parameter_defaults)
            settings.update(profile.motor_parameter_deviations.get(motor_name, {}))
            motor = Motor(name=motor_name, **settings)
            db.session.add(motor)
            stats["created"] += 1
        else:
            stats["updated"] += 1
        motor.controller = controllers[controller_number]
        motor.channel = channel

    obsolete_motors = Motor.query.filter(
        ~Motor.name.in_(tuple(profile.motor_mapping))
    ).all()
    obsolete_names = [motor.name for motor in obsolete_motors]
    if obsolete_names:
        referenced_names = [
            name
            for (name,) in db.session.query(MotorPosition.motor_name)
            .filter(MotorPosition.motor_name.in_(obsolete_names))
            .distinct()
            .order_by(MotorPosition.motor_name)
            .all()
        ]
        if referenced_names:
            warnings.append(
                "deleting motors still referenced by pose positions: "
                + ", ".join(referenced_names)
            )
        for motor in obsolete_motors:
            db.session.delete(motor)
        stats["deleted"] = len(obsolete_motors)

    db.session.flush()
    return stats, warnings


def _rebuild_button_programs(profile: HardwareProfile) -> None:
    ButtonProgram.query.delete(synchronize_session=False)
    db.session.flush()

    program = Program.query.filter_by(name="toggle_cerebra_fullscreen").first()
    program_id = program.id if program else None
    controllers = {
        controller.number: controller
        for controller in Controller.query.filter(
            Controller.number.in_(profile.rgb_button_controller_ids)
        ).all()
    }
    first, second, third = profile.rgb_button_controller_ids
    db.session.add_all(
        [
            ButtonProgram(controller_id=controllers[first].id, program_id=None),
            ButtonProgram(controller_id=controllers[second].id, program_id=None),
            ButtonProgram(controller_id=controllers[third].id, program_id=program_id),
        ]
    )
    db.session.flush()


def _delete_obsolete_controllers(
    profile: HardwareProfile, stats: dict[str, int]
) -> None:
    profile_numbers = tuple(controller.number for controller in profile.controllers)
    obsolete_controllers = Controller.query.filter(
        ~Controller.number.in_(profile_numbers)
    ).all()
    for controller in obsolete_controllers:
        motor_count = Motor.query.filter_by(controller_id=controller.id).count()
        button_count = ButtonProgram.query.filter_by(
            controller_id=controller.id
        ).count()
        if motor_count or button_count:
            raise RuntimeError(
                f"Cannot delete obsolete controller {controller.number}: "
                "it is still referenced."
            )
        db.session.delete(controller)
    stats["deleted"] = len(obsolete_controllers)
    db.session.flush()


def _orphaned_motor_position_names() -> list[str]:
    return [
        name
        for (name,) in db.session.query(MotorPosition.motor_name)
        .outerjoin(Motor, Motor.name == MotorPosition.motor_name)
        .filter(Motor.id.is_(None))
        .distinct()
        .order_by(MotorPosition.motor_name)
        .all()
    ]


@app.cli.command("reconcile_system_properties")
def reconcile_system_properties() -> None:
    """Mirror runtime facts while preserving an explicitly sourced DB variant."""
    set_property(SOFTWARE_VERSION_KEY, read_app_version(), "file")
    resolved_variant, resolved_source = resolve_variant_and_source()
    stored = get_property(HARDWARE_VARIANT_KEY)

    if stored is None:
        set_property(HARDWARE_VARIANT_KEY, resolved_variant, resolved_source)
    elif stored.value != resolved_variant and stored.source == "default":
        logger.info(
            "Updating default hardware variant from %s to %s (source: %s)",
            stored.value,
            resolved_variant,
            resolved_source,
        )
        set_property(HARDWARE_VARIANT_KEY, resolved_variant, resolved_source)
    elif stored.value != resolved_variant:
        logger.warning(
            "Resolved hardware variant %s (source: %s) differs from stored "
            "variant %s (source: %s); keeping the database value",
            resolved_variant,
            resolved_source,
            stored.value,
            stored.source,
        )
    db.session.commit()


def _is_empty_db() -> bool:
    inspector = inspect(db.engine)

    for table in inspector.get_table_names():
        if table in ("alembic_version", "system_property"):
            continue
        table_class = db.Model.metadata.tables.get(table)
        if table_class is not None:
            count = db.session.query(table_class).count()
            if count > 0:
                return False
    return True


def _create_controller_data(profile: HardwareProfile) -> None:
    controllers = [
        Controller(
            id=controller.number,
            number=controller.number,
            kind=controller.kind,
            device_type=controller.device_type,
            supply_voltage=controller.supply_voltage,
            address=controller.address,
        )
        for controller in profile.controllers
    ]
    db.session.add_all(controllers)
    db.session.flush()

    for motor_name, (controller_number, channel) in profile.motor_mapping.items():
        motor_settings = dict(profile.motor_parameter_defaults)
        motor_settings.update(profile.motor_parameter_deviations.get(motor_name, {}))
        motor = Motor(name=motor_name, **motor_settings)

        db.session.add(motor)
        db.session.flush()

        motor.controller = next(
            controller
            for controller in controllers
            if controller.number == controller_number
        )
        motor.channel = channel
        db.session.flush()


def _create_button_program_data(profile: HardwareProfile) -> None:
    cerebra_prog = Program.query.filter_by(name="toggle_cerebra_fullscreen").first()
    prog_id = cerebra_prog.id if cerebra_prog else None

    first_controller, second_controller, third_controller = (
        profile.rgb_button_controller_ids
    )
    button_program1 = ButtonProgram(controller_id=first_controller, program_id=None)
    button_program2 = ButtonProgram(controller_id=second_controller, program_id=None)
    button_program3 = ButtonProgram(controller_id=third_controller, program_id=prog_id)
    db.session.add_all([button_program1, button_program2, button_program3])
    db.session.flush()


def _create_camera_data() -> None:
    camera_settings = CameraSettings(
        resolution="SD", refresh_rate=0.1, quality_factor=80, res_x=640, res_y=480
    )
    db.session.add(camera_settings)
    db.session.flush()


def _create_program_data() -> None:
    program = Program(
        name="hello_world",
        code_visual=_get_example_program(),
        program_number="e1d46e2a-935e-4e2b-b2f9-0856af4257c5",
    )
    cerebra_program = Program(
        name="toggle_cerebra_fullscreen",
        code_visual='<xml xmlns="https://developers.google.com/blockly/xml"><block type="toggle_cerebra_fullscreen" id="cerebra_toggle" x="10" y="10"></block></xml>',
        program_number="c3r3br4-f-u-l-l-s-c-r-e-e-n-001",
    )
    db.session.add_all([program, cerebra_program])
    db.session.flush()


def _create_chat_data_and_assistant() -> None:
    gpt4o1 = AssistantModel(
        visual_name="GPT-4o [Vision]", api_name="gpt-4o", has_image_support=True
    )
    gpt4o2 = AssistantModel(
        visual_name="GPT-4o [Text]", api_name="gpt-4o", has_image_support=False
    )
    gpt3 = AssistantModel(
        visual_name="GPT-3.5 [Text]", api_name="gpt-3.5-turbo", has_image_support=False
    )
    claude = AssistantModel(
        visual_name="Claude 3 Sonnet [Vision]",
        api_name="anthropic.claude-3-sonnet-20240229-v1:0",
        has_image_support=True,
    )
    gemini_text = AssistantModel(
        visual_name="Gemini 3.5 Flash",
        api_name="gemini-3.5-flash",
        has_image_support=False,
    )
    hermes_agent = AssistantModel(
        visual_name="Hermes Agent (selbstlernend)",
        api_name="hermes-agent",
        has_image_support=True,
    )
    db.session.add_all([gpt4o2, gpt4o1, gpt3, claude, gemini_text, hermes_agent])
    db.session.flush()

    p_eva = Personality(
        name="Eva",
        personality_id="8f73b580-927e-41c2-98ac-e5df070e7288",
        gender="Female",
        pause_threshold=0.8,
        message_history=5,
        assistant_model_id=claude.id,
        stt_engine="local_whisper",
    )
    p_thomas = Personality(
        name="Thomas",
        personality_id="8b310f95-92cd-4512-b42a-d3fe29c4bb8a",
        gender="Male",
        pause_threshold=1.0,
        message_history=15,
        assistant_model_id=gpt4o1.id,
        stt_engine="local_whisper",
    )
    db.session.add_all([p_eva, p_thomas])
    db.session.flush()

    c1 = Chat(
        chat_id="b4f01552-0c09-401c-8fde-fda753fb0261",
        topic="Nuernberg",
        personality_id="8f73b580-927e-41c2-98ac-e5df070e7288",
    )
    c2 = Chat(
        chat_id="ee3e80f9-c8f7-48c2-9f15-449ba9bbe4ab",
        topic="Home-Office",
        personality_id="8b310f95-92cd-4512-b42a-d3fe29c4bb8a",
    )
    db.session.add_all([c1, c2])
    db.session.flush()

    m1 = ChatMessage(
        message_id="539ed3e6-9e3d-11ee-8c90-0242ac120002",
        is_user=True,
        content="hello pib!",
        chat_id="b4f01552-0c09-401c-8fde-fda753fb0261",
    )
    m2 = ChatMessage(
        message_id="0a080706-9e3e-11ee-8c90-0242ac120002",
        is_user=False,
        content="hello user!",
        chat_id="b4f01552-0c09-401c-8fde-fda753fb0261",
    )
    db.session.add_all([m1, m2])
    db.session.flush()


def _create_default_poses(profile: HardwareProfile) -> None:
    startup_pose = Pose(name=STARTUP_POSE_NAME, deletable=False)
    calibration_pose = Pose(name=CALIBRATION_POSE_NAME, deletable=False)

    db.session.add_all([startup_pose, calibration_pose])
    db.session.flush()

    startup_positions = [
        MotorPosition(
            position=STARTUP_POSITIONS.get(motor_name, 0),
            motor_name=motor_name,
            pose_id=startup_pose.id,
        )
        for motor_name in profile.motor_mapping
    ]

    calibration_positions = [
        MotorPosition(
            position=CALIBRATION_POSITIONS.get(motor_name, 0),
            motor_name=motor_name,
            pose_id=calibration_pose.id,
        )
        for motor_name in profile.motor_mapping
    ]

    db.session.add_all(startup_positions + calibration_positions)
    db.session.commit()


def _get_example_program() -> str:
    return """{"blocks":{"languageVersion":0,"blocks":[{"type":"text_print","id":"QWplsQn`*28S!rmDws$4","x":315,"y":279,"inputs":{"TEXT":{"shadow":{"type":"text","id":"`{AWS~jvKQo-ve^M@z-(","fields":{"TEXT":"hello world"}}}}}]}}"""
