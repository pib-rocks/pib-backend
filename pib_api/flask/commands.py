from collections import namedtuple
from typing import Any, Tuple

from sqlalchemy import inspect

from app.app import db, app
from model.bricklet_model import Bricklet
from model.bricklet_pin_model import BrickletPin
from model.camera_settings_model import CameraSettings
from model.chat_message_model import ChatMessage
from model.chat_model import Chat
from model.motor_model import Motor
from model.personality_model import Personality
from model.program_model import Program
from model.assistant_model import AssistantModel


@app.cli.command("seed_db")
def seed_db() -> None:
    if not _is_empty_db():
        print("Seeding database failed - database already contains data.")
        return
    _create_bricklet_data()
    _create_camera_data()
    _create_program_data()
    _create_chat_data_and_assistant()
    db.session.commit()
    print("Seeded the database with default data.")


def _is_empty_db() -> bool:
    inspector = inspect(db.engine)

    for table in inspector.get_table_names():
        if table == "alembic_version":
            continue
        table_class = db.Model.metadata.tables[table]
        count = db.session.query(table_class).count()
        if count > 0:
            return False
    return True


def _create_bricklet_data() -> None:
    data = _get_motor_list()
    motor_settings = {"pulse_width_min": 700, "pulse_width_max": 2500, "rotation_range_min": -9000,
                      "rotation_range_max": 9000, "velocity": 16000, "acceleration": 10000, "deceleration": 5000,
                      "period": 19500,
                      "turned_on": True, "visible": True, "invert": False}

    for item in data:
        motor = Motor(name=item["name"], **motor_settings)
        if motor.name == "tilt_forward_motor":
            motor.rotation_range_min = -4500
            motor.rotation_range_max = 4500
        elif motor.name == "tilt_sideways_motor":
            motor.visible = False
        # modify all fingers
        elif motor.name.endswith("stretch") or "thumb" in motor.name:
            motor.pulse_width_min = 750
            motor.velocity = 100000
            motor.acceleration = 50000
            motor.deceleration = 50000

        db.session.add(motor)
        db.session.flush()

        bricklet_pins: [Tuple[int, int]] = item["bricklet_pins"]
        for bricklet_pin in bricklet_pins:
            bricklet_id, pin = bricklet_pin

            invert = False
            if bricklet_pin == (3, 7) or bricklet_pin == (3, 5):
                invert = True
            db.session.add(BrickletPin(motor_id=motor.id, bricklet_id=bricklet_id, pin=pin, invert=invert))
        db.session.flush()

    b1 = Bricklet(uid="AAA", bricklet_number=1)
    b2 = Bricklet(uid="BBB", bricklet_number=2)
    b3 = Bricklet(uid="CCC", bricklet_number=3)
    db.session.add_all([b1, b2, b3])
    db.session.flush()


def _create_camera_data() -> None:
    camera_settings = CameraSettings(resolution="SD", refresh_rate=0.1, quality_factor=80, res_x=640, res_y=480)
    db.session.add(camera_settings)
    db.session.flush()


def _create_program_data() -> None:
    program = Program(name="hello_world", code_visual=_get_example_program())
    db.session.add(program)
    db.session.flush()


def _create_chat_data_and_assistant() -> None:
    gpt4 = AssistantModel(visual_name="GPT-4", api_name="gpt-4-turbo", has_image_support=False)
    gpt3 = AssistantModel(visual_name="GPT-3.5", api_name="gpt-3.5-turbo", has_image_support=False)
    claude = AssistantModel(visual_name="Claude 3 Sonnet", api_name="anthropic.claude-3-sonnet-20240229-v1:0", has_image_support=True)
    db.session.add_all([gpt3, gpt4, claude])
    db.session.flush()

    p_eva = Personality(name="Eva", personality_id="8f73b580-927e-41c2-98ac-e5df070e7288", gender="Female",
                        pause_threshold=0.8, assistant_model_id=claude.id)
    p_thomas = Personality(name="Thomas", personality_id="8b310f95-92cd-4512-b42a-d3fe29c4bb8a", gender="Male",
                           pause_threshold=0.8, assistant_model_id=gpt4.id)
    db.session.add_all([p_eva, p_thomas])
    db.session.flush()

    c1 = Chat(chat_id="b4f01552-0c09-401c-8fde-fda753fb0261", topic="Nuernberg",
              personality_id="8f73b580-927e-41c2-98ac-e5df070e7288")
    c2 = Chat(chat_id="ee3e80f9-c8f7-48c2-9f15-449ba9bbe4ab", topic="Home-Office",
              personality_id="8b310f95-92cd-4512-b42a-d3fe29c4bb8a")
    db.session.add_all([c1, c2])
    db.session.flush()

    m1 = ChatMessage(message_id="539ed3e6-9e3d-11ee-8c90-0242ac120002", is_user=True, content="hello pib!",
                     chat_id="b4f01552-0c09-401c-8fde-fda753fb0261")
    m2 = ChatMessage(message_id="0a080706-9e3e-11ee-8c90-0242ac120002", is_user=False, content="hello user!",
                     chat_id="b4f01552-0c09-401c-8fde-fda753fb0261")
    db.session.add_all([m1, m2])
    db.session.flush()


def _get_motor_list() -> [dict[str, Any]]:
    name: str = "name"
    bricklet_pins: str = "bricklet_pins"

    return [
        {name: "tilt_forward_motor", bricklet_pins: [(1, 0)]},
        {name: "tilt_sideways_motor", bricklet_pins: [(1, 1)]},
        {name: "turn_head_motor", bricklet_pins: [(2, 8)]},
        {name: "thumb_left_stretch", bricklet_pins: [(1, 9)]},
        {name: "thumb_left_opposition", bricklet_pins: [(2, 0)]},
        {name: "index_left_stretch", bricklet_pins: [(2, 1)]},
        {name: "middle_left_stretch", bricklet_pins: [(2, 2)]},
        {name: "ring_left_stretch", bricklet_pins: [(2, 3)]},
        {name: "pinky_left_stretch", bricklet_pins: [(2, 4)]},
        {name: "thumb_right_stretch", bricklet_pins: [(1, 3)]},
        {name: "thumb_right_opposition", bricklet_pins: [(1, 4)]},
        {name: "index_right_stretch", bricklet_pins: [(1, 5)]},
        {name: "middle_right_stretch", bricklet_pins: [(1, 6)]},
        {name: "ring_right_stretch", bricklet_pins: [(1, 7)]},
        {name: "pinky_right_stretch", bricklet_pins: [(1, 8)]},
        {name: "upper_arm_left_rotation", bricklet_pins: [(2, 5)]},
        {name: "elbow_left", bricklet_pins: [(2, 6)]},
        {name: "lower_arm_left_rotation", bricklet_pins: [(2, 7)]},
        {name: "wrist_left", bricklet_pins: [(1, 2)]},
        {name: "shoulder_vertical_left", bricklet_pins: [(3, 7), (3, 9)]},
        {name: "shoulder_horizontal_left", bricklet_pins: [(3, 9)]},
        {name: "upper_arm_right_rotation", bricklet_pins: [(3, 0)]},
        {name: "elbow_right", bricklet_pins: [(3, 1)]},
        {name: "lower_arm_right_rotation", bricklet_pins: [(3, 2)]},
        {name: "wrist_right", bricklet_pins: [(3, 4)]},
        {name: "shoulder_vertical_right", bricklet_pins: [(3, 8), (3, 5)]},
        {name: "shoulder_horizontal_right", bricklet_pins: [(3, 6)]},
    ]


def _get_example_program() -> str:
    return '''{"blocks":{"languageVersion":0,"blocks":[{"type":"text_print","id":"QWplsQn`*28S!rmDws$4","x":315,"y":279,"inputs":{"TEXT":{"shadow":{"type":"text","id":"`{AWS~jvKQo-ve^M@z-(","fields":{"TEXT":"hello world"}}}}}]}}'''
