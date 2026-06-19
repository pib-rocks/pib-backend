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

export const TF_BUTTON_TASTER_FUNCTION = (generator: CodeGenerator) => `
def ${generator.FUNCTION_NAME_PLACEHOLDER_}(button_id: int, red: int, green: int, blue: int) -> int:
    logging.info(f"reading Tinkerforge button {button_id} as taster.")

    color_request = SetButtonColor.Request()
    color_request.button_id = int(button_id)
    color_request.red = int(red)
    color_request.green = int(green)
    color_request.blue = int(blue)

    color_future = tf_button_set_color_client.call_async(color_request)
    rclpy.spin_until_future_complete(node, color_future)

    color_result = color_future.result()
    if color_result is None:
        raise RuntimeError("Tinkerforge button color service returned no result.")
    if not color_result.success:
        raise RuntimeError(color_result.message)

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

    color_request = SetButtonColor.Request()
    color_request.button_id = int(button_id)
    color_request.red = int(red)
    color_request.green = int(green)
    color_request.blue = int(blue)

    color_future = tf_button_set_color_client.call_async(color_request)
    rclpy.spin_until_future_complete(node, color_future)

    color_result = color_future.result()
    if color_result is None:
        raise RuntimeError("Tinkerforge button color service returned no result.")
    if not color_result.success:
        raise RuntimeError(color_result.message)

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

    request = SetButtonColor.Request()
    request.button_id = int(button_id)
    request.red = int(red)
    request.green = int(green)
    request.blue = int(blue)

    future = tf_button_set_color_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    result = future.result()
    if result is None:
        raise RuntimeError("Tinkerforge button color service returned no result.")
    if not result.success:
        raise RuntimeError(result.message)
`;
