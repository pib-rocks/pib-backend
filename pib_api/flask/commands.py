"""Flask CLI commands.

Docker Compose runs ``seed_db`` when the API container starts. A selected hardware
variant without an implemented profile therefore fails startup loudly instead of
silently seeding hardware for a different robot.
"""

from sqlalchemy import inspect

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
    get_profile,
    resolve_variant_from_environment,
)
from default_pose_constants import (
    STARTUP_POSITIONS,
    CALIBRATION_POSITIONS,
    STARTUP_POSE_NAME,
    CALIBRATION_POSE_NAME,
)
from model.button_program_model import ButtonProgram


@app.cli.command("seed_db")
def seed_db() -> None:
    if not _is_empty_db():
        print("Seeding database failed - database already contains data.")
        return
    variant = resolve_variant_from_environment()
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
    db.session.commit()
    print("Seeded the database with default data.")


def _is_empty_db() -> bool:
    inspector = inspect(db.engine)

    for table in inspector.get_table_names():
        if table == "alembic_version":
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
