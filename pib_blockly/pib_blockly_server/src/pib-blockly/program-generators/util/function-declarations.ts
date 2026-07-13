import {CodeGenerator} from "blockly";

// play-audio-from-speech

export const PLAY_AUDIO_FROM_SPEECH_FUNCTION = (generator: CodeGenerator) => `
def ${generator.FUNCTION_NAME_PLACEHOLDER_}(speech: str, voice: str) -> None:

    logging.info(f"received request to say '{speech}' as '{voice}'.")

    request = PlayAudioFromSpeech.Request()
    request.speech = speech
    request.join = True

    if voice == 'Hannah':
        request.gender = "Female"
        request.language = "German"
    elif voice == 'Daniel':
        request.gender = "Male"
        request.language = "German"
    elif voice == 'Emma':
        request.gender = "Female"
        request.language = "English"
    elif voice == 'Brian':
        request.gender = "Male"
        request.language = "English"
    else:
        logging.error(f"unrecognized voice: '{voice}', aborting...")
        return

    future = play_audio_from_speech_client.call_async(request)

    logging.info(f"now speaking...")
    rclpy.spin_until_future_complete(node, future)
    logging.info("finished speaking.")
`;

export const PLAY_AUDIO_FROM_FILE_FUNCTION = (generator: CodeGenerator) => `
def ${generator.FUNCTION_NAME_PLACEHOLDER_}(filepath: str) -> None:

    logging.info(f"received request to play audio file '{filepath}'.")

    request = PlayAudioFromFile.Request()
    request.filepath = filepath
    request.join = True

    future = play_audio_from_file_client.call_async(request)

    logging.info(f"now playing audio file...")
    rclpy.spin_until_future_complete(node, future)
    logging.info("finished playing audio file.")
`;

// motor

export const GET_JOINT_POSITION_FUNCTION = (generator: CodeGenerator) => `
def ${generator.FUNCTION_NAME_PLACEHOLDER_}(motor_name: str) -> int:

    request = GetJointPosition.Request()
    request.joint_name = motor_name

    future = get_joint_position_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    response: GetJointPosition.Response = future.result()
    if response.successful:
        return response.position
    else:
        logging.error(f"getting position of '{motor_name}' failed.")
        return 0
`;

export const APPLY_JOINT_TRAJECTORY_FUNCTION = (generator: CodeGenerator) => `
def ${generator.FUNCTION_NAME_PLACEHOLDER_}(motor_name: str, position: int) -> None:

    logging.info(f"setting position of '{motor_name}' to {position}.")

    request = ApplyJointTrajectory.Request()
    point = JointTrajectoryPoint()
    point.positions.append(position)
    jt = JointTrajectory()
    jt.joint_names = [motor_name]
    jt.points = [point]
    request.joint_trajectory = jt

    future = apply_joint_trajectory_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    response: ApplyJointTrajectory.Response = future.result()
    if response.successful:
        logging.info(f"position of '{motor_name}' was successfully set.")
    else:
        logging.error(f"setting position of '{motor_name}' failed.")
`;

// pose

export const APPLY_POSE_FUNCTION = (generator: CodeGenerator) => `
def ${generator.FUNCTION_NAME_PLACEHOLDER_}(poseId: str) -> None:

    logging.info(f"Pose ID: {poseId}")
    logging.info(f"moving to pose..")

    successful, motor_positions = pose_client.get_motor_positions_of_pose(
            poseId
        )
    if not successful:
        logging.error(f"getting motor-positions of pose failed.")
        return

    jt = JointTrajectory()
    jt.joint_names = []

    for motor_position in motor_positions["motorPositions"]:
        motor_name = motor_position["motorName"]
        position = motor_position["position"]

        jt.joint_names.append(motor_name)
        point = JointTrajectoryPoint()
        point.positions.append(position)
        jt.points.append(point)

    request = ApplyJointTrajectory.Request()
    request.joint_trajectory = jt

    future = apply_joint_trajectory_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    response: ApplyJointTrajectory.Response = future.result()
    if response.successful:
        logging.info(f"pose was successfully applied.")
    else:
        logging.error(f"applying pose failed.")
`;

// set-solid-state-relay

export const SET_SOLID_STATE_RELAY_FUNCTION = (generator: CodeGenerator) => `

def ${generator.FUNCTION_NAME_PLACEHOLDER_}(status: str) -> None:

    state = status == 'ON'

    logging.info(f"received request to turn solid state relay to '{status}'.")
    request = SetSolidStateRelay.Request()
    request.solid_state_relay_state = SolidStateRelayState(turned_on=state)

    future = set_solid_state_relay_state_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    response: SetSolidStateRelay.Response = future.result()
    if response.successful:
        logging.info(f"solid state relay was successfully set to '{status}'.")
    else:
        logging.error(f"setting solid state relay failed.")
`;

// face-detector

export const FACE_DETECTOR_CLASS = (generator: CodeGenerator) => `
import os
import rclpy
from std_msgs.msg import Float32MultiArray

class ${generator.FUNCTION_NAME_PLACEHOLDER_}():

    def __init__(self):
        self.x_center = 0.0
        self.y_center = 0.0
        self.node = None
        self.subscription = None

        if not rclpy.ok():
            rclpy.init(args=None)

        self.node = rclpy.create_node(f"blockly_face_detector_{os.getpid()}")
        self.subscription = self.node.create_subscription(
            Float32MultiArray,
            "/face_center",
            self.face_center_callback,
            10
        )

    def face_center_callback(self, msg):
        if len(msg.data) >= 2:
            self.x_center = float(msg.data[0])
            self.y_center = float(msg.data[1])
        else:
            self.x_center = 0.0
            self.y_center = 0.0

    def updateDetector(self):
        if self.node is not None:
            rclpy.spin_once(self.node, timeout_sec=0.02)

        return (self.x_center, self.y_center)

    def close(self):
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
`;

// vision

export const VISION_HELPER_CLASS = (generator: CodeGenerator) => `
import os
import rclpy
from datatypes.srv import VisionPrompt

class ${generator.FUNCTION_NAME_PLACEHOLDER_}():

    def __init__(self):
        if not rclpy.ok():
            rclpy.init(args=None)

        self.node = rclpy.create_node(f"blockly_vision_helper_{os.getpid()}")
        self.client = self.node.create_client(VisionPrompt, "vision_prompt")
        if not self.client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("vision_prompt service is not available")

    def ask(self, prompt: str) -> str:
        request = VisionPrompt.Request()
        request.prompt = str(prompt)

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=60.0)

        if not future.done():
            raise RuntimeError("vision_prompt service call timed out")

        result = future.result()
        if result is None:
            return ""

        return str(result.response).strip()

    def _to_bool_number(self, response: str) -> int:
        cleaned = str(response).strip().lower()

        if cleaned.startswith("1"):
            return 1

        if cleaned.startswith("0"):
            return 0

        if cleaned in ["ja", "yes", "true", "wahr"]:
            return 1

        return 0

    def object_detected(self, object_name: str) -> int:
        prompt = (
            "Check the current camera image. "
            f"Is at least one object visible that matches this description: '{object_name}'? "
            "The description may be in English or German. "
            "Answer only with 1 for yes or 0 for no."
        )
        return self._to_bool_number(self.ask(prompt))

    def object_count(self, object_name: str, count: int) -> int:
        prompt = (
            "Check the current camera image. "
            f"Are at least {int(count)} objects visible that match this description: '{object_name}'? "
            "The description may be in English or German. "
            "Answer only with 1 for yes or 0 for no."
        )
        return self._to_bool_number(self.ask(prompt))

    def objects_different(self, object_a: str, object_b: str) -> int:
        prompt = (
            "Check the current camera image. "
            f"Are two shown objects visible where one matches '{object_a}' and one matches '{object_b}', "
            "and are they different object types? "
            "The descriptions may be in English or German. "
            "Examples: apple and banana returns 1. apple and apple returns 0. "
            "Answer only with 1 for yes or 0 for no."
        )
        return self._to_bool_number(self.ask(prompt))

    def describe_image(self, language: str = "de") -> str:
        if str(language).lower().startswith("en"):
            prompt = (
                "Describe the current camera image briefly and factually in one single English sentence."
            )
        else:
            prompt = (
                "Beschreibe das aktuelle Kamerabild kurz und sachlich in einem einzigen deutschen Satz."
            )

        return self.ask(prompt)

    def close(self):
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
`;



export const TF_BUTTON_TASTER_FUNCTION = (generator: CodeGenerator) => `
def ${generator.FUNCTION_NAME_PLACEHOLDER_}(button_id: int, red: int, green: int, blue: int) -> int:
    logging.info(f"reading Tinkerforge button {button_id} as taster.")

    # Use consolidated topic API; do not persist color for read helpers.
    blockly_client.set_button_color(button_id, red, green, blue, sticky=False)

    read_request = ReadButton.Request()
    read_request.button_id = int(button_id)

    read_future = tf_button_read_client.call_async(read_request)
    rclpy.spin_until_future_complete(node, read_future)

    read_result = read_future.result()
    if read_result is None:
        raise RuntimeError("Tinkerforge button read service returned no result.")
    if not read_result.success:
        raise RuntimeError(read_result.message)

    return 1 if read_result.pressed else 0
`;

export const TF_BUTTON_SWITCH_FUNCTION = (generator: CodeGenerator) => `
def ${generator.FUNCTION_NAME_PLACEHOLDER_}(button_id: int, red: int, green: int, blue: int) -> int:
    logging.info(f"reading Tinkerforge button {button_id} as switch.")

    # Use consolidated topic API; do not persist color for read helpers.
    blockly_client.set_button_color(button_id, red, green, blue, sticky=False)

    read_request = ReadButton.Request()
    read_request.button_id = int(button_id)

    read_future = tf_button_read_client.call_async(read_request)
    rclpy.spin_until_future_complete(node, read_future)

    read_result = read_future.result()
    if read_result is None:
        raise RuntimeError("Tinkerforge button read service returned no result.")
    if not read_result.success:
        raise RuntimeError(read_result.message)

    return 1 if read_result.switched_on else 0
`;

export const TF_BUTTON_SET_COLOR_FUNCTION = (generator: CodeGenerator) => `
def ${generator.FUNCTION_NAME_PLACEHOLDER_}(button_id: int, red: int, green: int, blue: int) -> None:
    logging.info(f"setting Tinkerforge button {button_id} color to rgb({red}, {green}, {blue}).")

    # Consolidated API: publish sticky color until overwritten.
    blockly_client.set_button_color(button_id, red, green, blue)
`;
