import { pythonGenerator } from "blockly/python";

// play-audio-from-speech

export const PLAY_AUDIO_FROM_SPEECH_FUNCTION = (
  generator: typeof pythonGenerator,
) => `
def ${generator.FUNCTION_NAME_PLACEHOLDER_}(speech: str, voice: str = "F1", language: str = "auto") -> None:

    logging.info(f"received request to say '{speech}' with voice '{voice}' in language '{language}'.")

    request = PlayAudioFromSpeech.Request()
    request.speech = speech
    request.gender = voice
    request.language = language
    request.join = True

    future = play_audio_from_speech_client.call_async(request)

    logging.info(f"now speaking via local Supertonic TTS...")
    rclpy.spin_until_future_complete(node, future)
    logging.info("finished speaking.")
`;

export const PLAY_AUDIO_FROM_FILE_FUNCTION = (
  generator: typeof pythonGenerator,
) => `
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

export const SET_VOLUME_FUNCTION = (generator: typeof pythonGenerator) => `
def ${generator.FUNCTION_NAME_PLACEHOLDER_}(percent: int) -> None:

    request = SetVolume.Request()
    request.percent = int(percent)

    future = set_volume_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
`;

// motor

/** ROS GetJointPosition and pose HTTP APIs store angles as centidegrees; Blockly uses degrees. */
export const CENTIDEGREES_TO_DEGREES_FUNCTION = (
  generator: typeof pythonGenerator,
) => `
def ${generator.FUNCTION_NAME_PLACEHOLDER_}(position_centidegrees: int) -> float:
    return position_centidegrees / 100.0
`;

export const GET_JOINT_POSITION_FUNCTION = (
  generator: typeof pythonGenerator,
) => `
def ${generator.FUNCTION_NAME_PLACEHOLDER_}(motor_name: str) -> float:

    request = GetJointPosition.Request()
    request.joint_name = motor_name

    future = get_joint_position_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    response: GetJointPosition.Response = future.result()
    if response.successful:
        return centidegrees_to_degrees(response.position)
    else:
        logging.error(f"getting position of '{motor_name}' failed.")
        return 0.0
`;

export const SET_HAND_POSITION_XYZ_FUNCTION = (
  generator: typeof pythonGenerator,
) => `
def ${generator.FUNCTION_NAME_PLACEHOLDER_}(side: str, mode: str, x, y, z) -> None:

    if mode == "RELATIVE":
        current = get_hand_position_xyz(side)
        offset = list(from_rep103([x, y, z]))
        target = [
            current[0] + offset[0],
            current[1] + offset[1],
            current[2] + offset[2],
        ]
    else:
        target = list(from_rep103([x, y, z]))

    logging.info(f"setting {side} hand position to xyz={target}.")
    q_deg = ik_deg(side, xyz=target)
    arm = left_arm if side == "left" else right_arm
    rosbridge_host = os.getenv("ROSBRIDGE_HOST", "rosbridge-ws")
    try:
        with Write(host=rosbridge_host, port=9090) as pib:
            pib.move(arm, *q_deg)
    except Exception:
        with Write(host="localhost", port=9090) as pib:
            pib.move(arm, *q_deg)
`;

export const GET_HAND_POSITION_REP103_FUNCTION = (
  generator: typeof pythonGenerator,
) => `
def ${generator.FUNCTION_NAME_PLACEHOLDER_}(side: str):

    rosbridge_host = os.getenv("ROSBRIDGE_HOST", "rosbridge-ws")
    try:
        return to_rep103(get_hand_position_xyz(side, telemetry_host=rosbridge_host))
    except Exception:
        return to_rep103(get_hand_position_xyz(side, telemetry_host="localhost"))
`;

export const APPLY_JOINT_TRAJECTORY_FUNCTION = (
  generator: typeof pythonGenerator,
) => `
def ${generator.FUNCTION_NAME_PLACEHOLDER_}(motor_name: str, position) -> None:

    logging.info(f"setting position of '{motor_name}' to {position} degrees.")
    rosbridge_host = os.getenv("ROSBRIDGE_HOST", "rosbridge-ws")
    try:
        pib_sdk.Write(host=rosbridge_host, port=9090).move(motor_name, position)
    except Exception:
        pib_sdk.Write(host="localhost", port=9090).move(motor_name, position)
`;

export const MOVE_MOTORS_FUNCTION = (generator: typeof pythonGenerator) => `
def ${generator.FUNCTION_NAME_PLACEHOLDER_}(*args) -> None:

    if not args:
        return

    logging.info(f"moving motors in one joint trajectory: {args}.")
    rosbridge_host = os.getenv("ROSBRIDGE_HOST", "rosbridge-ws")
    try:
        with pib_sdk.Write(host=rosbridge_host, port=9090) as write:
            write.move(*args)
    except Exception:
        with pib_sdk.Write(host="localhost", port=9090) as write:
            write.move(*args)
`;

export const GET_MOTOR_CURRENT_MA_FUNCTION = (
  generator: typeof pythonGenerator,
) => `
def ${generator.FUNCTION_NAME_PLACEHOLDER_}(motor_name: str) -> int:

    rosbridge_host = os.getenv("ROSBRIDGE_HOST", "rosbridge-ws")
    try:
        with Telemetry(host=rosbridge_host, port=9090) as telemetry:
            return telemetry.get_current_ma(motor_name)
    except Exception:
        with Telemetry(host="localhost", port=9090) as telemetry:
            return telemetry.get_current_ma(motor_name)
`;

export const GET_CAMERA_DEPTH_FRAME_FUNCTION = (
  generator: typeof pythonGenerator,
) => `
def ${generator.FUNCTION_NAME_PLACEHOLDER_}():

    rosbridge_host = os.getenv("ROSBRIDGE_HOST", "rosbridge-ws")
    try:
        with Camera(host=rosbridge_host, port=9090) as cam:
            return cam.get_depth_frame()
    except Exception:
        with Camera(host="localhost", port=9090) as cam:
            return cam.get_depth_frame()
`;

export const GET_CAMERA_DISTANCE_AT_PX_FUNCTION = (
  generator: typeof pythonGenerator,
) => `
def ${generator.FUNCTION_NAME_PLACEHOLDER_}(x, y):

    rosbridge_host = os.getenv("ROSBRIDGE_HOST", "rosbridge-ws")
    try:
        with Camera(host=rosbridge_host, port=9090) as cam:
            return cam.get_distance_at_px(int(x), int(y))
    except Exception:
        with Camera(host="localhost", port=9090) as cam:
            return cam.get_distance_at_px(int(x), int(y))
`;

export const GET_IMU_ACCELERATION_FUNCTION = (
  generator: typeof pythonGenerator,
) => `
def ${generator.FUNCTION_NAME_PLACEHOLDER_}(axis):

    rosbridge_host = os.getenv("ROSBRIDGE_HOST", "rosbridge-ws")
    try:
        with IMU(host=rosbridge_host, port=9090) as imu:
            data = imu.latest()
            if data is None:
                return 0.0
            acc = data.acceleration_m_s2
            if axis == "total":
                return math.sqrt(acc.x ** 2 + acc.y ** 2 + acc.z ** 2)
            return getattr(acc, axis)
    except Exception:
        try:
            with IMU(host="localhost", port=9090) as imu:
                data = imu.latest()
                if data is None:
                    return 0.0
                acc = data.acceleration_m_s2
                if axis == "total":
                    return math.sqrt(acc.x ** 2 + acc.y ** 2 + acc.z ** 2)
                return getattr(acc, axis)
        except Exception:
            return 0.0
`;

export const GET_IMU_ANGULAR_VELOCITY_FUNCTION = (
  generator: typeof pythonGenerator,
) => `
def ${generator.FUNCTION_NAME_PLACEHOLDER_}(axis):

    rosbridge_host = os.getenv("ROSBRIDGE_HOST", "rosbridge-ws")
    try:
        with IMU(host=rosbridge_host, port=9090) as imu:
            data = imu.latest()
            if data is None:
                return 0.0
            return getattr(data.angular_velocity_rad_s, axis)
    except Exception:
        try:
            with IMU(host="localhost", port=9090) as imu:
                data = imu.latest()
                if data is None:
                    return 0.0
                return getattr(data.angular_velocity_rad_s, axis)
        except Exception:
            return 0.0
`;

export const GET_IMU_IS_DATA_AVAILABLE_FUNCTION = (
  generator: typeof pythonGenerator,
) => `
def ${generator.FUNCTION_NAME_PLACEHOLDER_}():

    rosbridge_host = os.getenv("ROSBRIDGE_HOST", "rosbridge-ws")
    try:
        with IMU(host=rosbridge_host, port=9090) as imu:
            return imu.latest() is not None
    except Exception:
        try:
            with IMU(host="localhost", port=9090) as imu:
                return imu.latest() is not None
        except Exception:
            return False
`;

export const GET_IMU_DATA_AGE_FUNCTION = (
  generator: typeof pythonGenerator,
) => `
def ${generator.FUNCTION_NAME_PLACEHOLDER_}():

    rosbridge_host = os.getenv("ROSBRIDGE_HOST", "rosbridge-ws")
    try:
        with IMU(host=rosbridge_host, port=9090) as imu:
            data = imu.latest()
            if data is None:
                return 0.0
            return data.age_s
    except Exception:
        try:
            with IMU(host="localhost", port=9090) as imu:
                data = imu.latest()
                if data is None:
                    return 0.0
                return data.age_s
        except Exception:
            return 0.0
`;

// pose

export const APPLY_POSE_FUNCTION = (generator: typeof pythonGenerator) => `
def ${generator.FUNCTION_NAME_PLACEHOLDER_}(poseId: str) -> None:

    logging.info(f"Pose ID: {poseId}")
    logging.info(f"moving to pose..")

    successful, motor_positions = pose_client.get_motor_positions_of_pose(
            poseId
        )
    if not successful:
        logging.error(f"getting motor-positions of pose failed.")
        return

    args = []
    for motor_position in motor_positions["motorPositions"]:
        args.extend(
            (
                motor_position["motorName"],
                centidegrees_to_degrees(motor_position["position"]),
            )
        )
    if not args:
        return

    rosbridge_host = os.getenv("ROSBRIDGE_HOST", "rosbridge-ws")
    try:
        with pib_sdk.Write(host=rosbridge_host, port=9090) as writer:
            writer.move(*args)
    except Exception:
        with pib_sdk.Write(host="localhost", port=9090) as writer:
            writer.move(*args)
`;

export const SAVE_CURRENT_POSE_FUNCTION = (
  generator: typeof pythonGenerator,
) => `
def ${generator.FUNCTION_NAME_PLACEHOLDER_}(name: str) -> None:

    motor_names = _expand_motor_specs([All])
    assert "turn_head_motor" in motor_names
    assert "tilt_forward_motor" in motor_names

    with Telemetry(host=rosbridge_host, port=9090) as telemetry:
        save_current_pose(telemetry, pose_backend, name, motor_names)
`;

export const PLAY_POSE_SEQUENCE_TIMED_FUNCTION = (
  generator: typeof pythonGenerator,
) => `
def ${generator.FUNCTION_NAME_PLACEHOLDER_}(sequence) -> None:
    steps = [
        (str(item[0]), float(item[1]))
        for item in (sequence or [])
        if item and len(item) >= 2 and str(item[0]).strip() and str(item[0]).strip() != "LOADING"
    ]
    if not steps:
        return

    resolved_poses = []
    for identifier, leg_seconds in steps:
        pose = None
        try:
            pose = get_pose(pose_backend, pose_id=identifier)
        except Exception:
            pose = get_pose(pose_backend, name=identifier)
        resolved_poses.append((pose, max(0.05, float(leg_seconds))))

    joint_names = []
    seen = set()
    for pose, _leg_seconds in resolved_poses:
        for motor_name in pose.motor_angles_deg:
            if motor_name not in seen:
                seen.add(motor_name)
                joint_names.append(motor_name)
    if not joint_names:
        return

    filled = []
    previous = {}
    for pose, _leg_seconds in resolved_poses:
        current = dict(previous)
        for k, v in pose.motor_angles_deg.items():
            limit = 180.0 if ("stretch" in k or "opposition" in k) else 90.0
            current[k] = min(limit, max(-limit, float(v)))
        filled.append(current)
        previous = current

    first_known = {}
    for angle_map in filled:
        for motor_name, angle_deg in angle_map.items():
            first_known.setdefault(motor_name, float(angle_deg))
    for angle_map in filled:
        for motor_name in joint_names:
            angle_map.setdefault(motor_name, first_known[motor_name])

    initial_angles = dict(first_known)
    for host in (rosbridge_host, "localhost"):
        try:
            with Telemetry(host=host, port=9090) as telemetry:
                batch_loaded = False
                try:
                    import roslibpy as _roslibpy
                    _srv = _roslibpy.Service(
                        telemetry.ros,
                        "/get_all_joint_positions",
                        "datatypes/srv/GetAllJointPositions",
                    )
                    _resp = _srv.call(
                        _roslibpy.ServiceRequest({"joint_names": joint_names}),
                        timeout=3.0,
                    )
                    for _res, _pos_cdeg in zip(
                        _resp.get("results", []), _resp.get("positions", [])
                    ):
                        if _res.get("successful"):
                            _m = _res["motor_name"]
                            _lim = 180.0 if ("stretch" in _m or "opposition" in _m) else 90.0
                            initial_angles[_m] = min(_lim, max(-_lim, float(_pos_cdeg) / 100.0))
                            batch_loaded = True
                except Exception:
                    batch_loaded = False
                if not batch_loaded:
                    live_joints = telemetry.get_positions_deg(joint_names)
                    for motor_name, angle_deg in (live_joints or {}).items():
                        if angle_deg is not None:
                            limit = 180.0 if ("stretch" in motor_name or "opposition" in motor_name) else 90.0
                            initial_angles[motor_name] = min(limit, max(-limit, float(angle_deg)))
                break
        except Exception:
            continue

    active_joints = [
        motor_name
        for motor_name in joint_names
        if max([initial_angles[motor_name]] + [m[motor_name] for m in filled])
        - min([initial_angles[motor_name]] + [m[motor_name] for m in filled]) >= 0.2
    ]
    if active_joints:
        joint_names = active_joints
    else:
        joint_names = [joint_names[0]]

    prev_cdeg = [float(round(initial_angles[motor_name] * 100.0)) for motor_name in joint_names]
    waypoints = [(prev_cdeg, 0.0)]
    time_from_start = 0.0
    for step_idx, ((_pose, leg_seconds), angle_map) in enumerate(zip(resolved_poses, filled)):
        target_cdeg = [float(round(angle_map[motor_name] * 100.0)) for motor_name in joint_names]
        max_delta = max((abs(p_to - p_from) for p_from, p_to in zip(prev_cdeg, target_cdeg)), default=0.0)
        if step_idx == 0 and max_delta < 200.0:
            waypoints[0] = (target_cdeg, 0.0)
            prev_cdeg = target_cdeg
            continue
        min_phys_sec = 0.0
        for motor_name, p_from, p_to in zip(joint_names, prev_cdeg, target_cdeg):
            delta = abs(p_to - p_from)
            cap = 22000.0 if ("stretch" in motor_name or "opposition" in motor_name) else 10000.0
            min_phys_sec = max(min_phys_sec, delta / cap)
        eff_seconds = max(leg_seconds, min_phys_sec)
        time_from_start = round(time_from_start + eff_seconds, 4)
        waypoints.append((target_cdeg, time_from_start))
        prev_cdeg = target_cdeg

    if len(waypoints) < 2:
        waypoints.append((prev_cdeg, 0.05))
        time_from_start = 0.05

    for host in (rosbridge_host, "localhost"):
        try:
            with pib_sdk.Write(host=host, port=9090) as writer:
                if hasattr(writer, "send_timed_trajectory"):
                    writer.send_timed_trajectory(joint_names, waypoints)
                else:
                    play_pose_sequence_timed(writer, pose_backend, steps)
                break
        except Exception:
            if host == "localhost":
                raise
            continue

    if time_from_start > 0.0:
        time_from_start = round(time_from_start + 0.08, 4)
        time.sleep(time_from_start)
`;

// model inference

export const START_MODEL_FUNCTION = (generator: typeof pythonGenerator) => `
def ${generator.FUNCTION_NAME_PLACEHOLDER_}(model_id) -> None:
    rosbridge_host = os.getenv("ROSBRIDGE_HOST", "rosbridge-ws")
    with Models(host=rosbridge_host, port=9090) as models:
        models.start_model(str(model_id))
`;

export const STOP_MODEL_FUNCTION = (generator: typeof pythonGenerator) => `
def ${generator.FUNCTION_NAME_PLACEHOLDER_}(model_id) -> None:
    rosbridge_host = os.getenv("ROSBRIDGE_HOST", "rosbridge-ws")
    with Models(host=rosbridge_host, port=9090) as models:
        models.stop_model(str(model_id))
`;

export const STOP_ALL_MODELS_FUNCTION = (generator: typeof pythonGenerator) => `
def ${generator.FUNCTION_NAME_PLACEHOLDER_}() -> None:
    rosbridge_host = os.getenv("ROSBRIDGE_HOST", "rosbridge-ws")
    with Models(host=rosbridge_host, port=9090) as models:
        models.stop_all_models()
`;

function latestDetectionsFunction(
  generator: typeof pythonGenerator,
  topic: string,
  emptyWarning: string,
  extraFields = "",
  extraHelpers = "",
) {
  return `
def ${generator.FUNCTION_NAME_PLACEHOLDER_}(timeout_sec=10.0):
    received = {}

    def _on_detection(message):
        received["message"] = message

    subscription = node.create_subscription(
        DetectionArray,
        "${topic}",
        _on_detection,
        10,
    )
    deadline = time.monotonic() + timeout_sec
    try:
        while "message" not in received and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_subscription(subscription)

    message = received.get("message")
    if message is None:
        logging.warning("${emptyWarning}")
        return []
${extraHelpers}
    return [
        [
            detection.label,
            detection.score,
            detection.x_min,
            detection.y_min,
            detection.x_max,
            detection.y_max,${extraFields}
        ]
        for detection in message.detections
    ]
`;
}

export const GET_FACE_DETECTIONS_FUNCTION = (
  generator: typeof pythonGenerator,
) =>
  latestDetectionsFunction(
    generator,
    "/detections/face_detection_yunet_160x120",
    "no face detections received",
  );

export const GET_OBJECT_DETECTIONS_FUNCTION = (
  generator: typeof pythonGenerator,
) =>
  latestDetectionsFunction(
    generator,
    "/detections/yolov6n_coco_640x640",
    "no object detections received",
  );

export const GET_QR_DETECTIONS_FUNCTION = (generator: typeof pythonGenerator) =>
  latestDetectionsFunction(
    generator,
    "/detections/qr_code_detection_384x384",
    "no qr detections received",
  );

export const GET_EMOTION_DETECTIONS_FUNCTION = (
  generator: typeof pythonGenerator,
) =>
  latestDetectionsFunction(
    generator,
    "/detections/emotion_recognition_crop",
    "no emotion detections received",
  );

export const GET_HEAD_POSE_DETECTIONS_FUNCTION = (
  generator: typeof pythonGenerator,
) =>
  latestDetectionsFunction(
    generator,
    "/detections/head_pose_estimation_crop",
    "no head pose detections received",
    `
            _scalar(detection, "yaw_deg", "yaw"),
            _scalar(detection, "pitch_deg", "pitch"),
            _scalar(detection, "roll_deg", "roll"),`,
    `
    def _scalar(detection, *names):
        for name in names:
            if name in detection.scalar_names:
                index = detection.scalar_names.index(name)
                if index < len(detection.scalar_values):
                    return detection.scalar_values[index]
        return 0.0
`,
  );

export const IMITATE_HANDS_FUNCTION = (generator: typeof pythonGenerator) => `
_imitate_hands_state = {
    "sub": None,
    "msg": None,
    "seq": 0,
    "last_seq": -1,
    "future": None,
    "last_cdeg": {},
    "no_hand_sent": False,
}

def ${generator.FUNCTION_NAME_PLACEHOLDER_}(side: str = "both", parts: str = "all") -> None:
    if _imitate_hands_state["sub"] is None:

        def _on_hand_msg(msg):
            _imitate_hands_state["msg"] = msg
            _imitate_hands_state["seq"] += 1

        _imitate_hands_state["sub"] = node.create_subscription(
            DetectionArray,
            "/detections/hand_tracking_fast",
            _on_hand_msg,
            10,
        )
        deadline = time.monotonic() + 0.5
        while _imitate_hands_state["msg"] is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

    rclpy.spin_once(node, timeout_sec=0.02)
    if _imitate_hands_state["seq"] == _imitate_hands_state["last_seq"]:
        spin_deadline = time.monotonic() + 0.08
        while (
            _imitate_hands_state["seq"] == _imitate_hands_state["last_seq"]
            and time.monotonic() < spin_deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.01)

    if _imitate_hands_state["msg"] is None:
        return
    if _imitate_hands_state["seq"] == _imitate_hands_state["last_seq"]:
        return
    if (
        _imitate_hands_state["future"] is not None
        and not _imitate_hands_state["future"].done()
    ):
        return

    _imitate_hands_state["last_seq"] = _imitate_hands_state["seq"]
    msg = _imitate_hands_state["msg"]
    fw = float(msg.frame_width or 1280.0)
    fh = float(msg.frame_height or 720.0)

    valid_hands = [
        det
        for det in msg.detections
        if len(det.keypoint_x) >= 21
        and len(det.keypoint_y) >= 21
        and len(det.keypoint_z) >= 21
    ]

    def _handedness(det):
        if "handedness" in det.scalar_names:
            index = det.scalar_names.index("handedness")
            if index < len(det.scalar_values):
                return float(det.scalar_values[index])
        return 0.5

    def _clamp_joint(name, val):
        if name in ("elbow_left", "elbow_right"):
            return max(-4500.0, min(9000.0, val))
        return max(-9000.0, min(9000.0, val))

    def _bend_angle(pts, i0, i1, i2, i3):
        v1 = (pts[i0][0] - pts[i1][0], pts[i0][1] - pts[i1][1], pts[i0][2] - pts[i1][2])
        v2 = (pts[i2][0] - pts[i3][0], pts[i2][1] - pts[i3][1], pts[i2][2] - pts[i3][2])
        n1 = math.sqrt(v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2])
        n2 = math.sqrt(v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2])
        if n1 < 1e-6 or n2 < 1e-6:
            return 0.0
        cos_a = max(
            -1.0,
            min(1.0, (v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]) / (n1 * n2)),
        )
        return math.acos(cos_a)

    targets = {}

    if len(valid_hands) == 0:
        if parts in ("all", "arms") and not _imitate_hands_state["no_hand_sent"]:
            rest_left = {
                "shoulder_vertical_left": -7900.0,
                "upper_arm_left_rotation": 0.0,
                "elbow_left": 0.0,
                "lower_arm_left_rotation": 0.0,
                "shoulder_horizontal_left": 9000.0,
            }
            rest_right = {
                "shoulder_vertical_right": -7300.0,
                "upper_arm_right_rotation": 0.0,
                "elbow_right": 500.0,
                "lower_arm_right_rotation": -1100.0,
                "shoulder_horizontal_right": 7500.0,
            }
            if side in ("both", "left"):
                targets.update(rest_left)
            if side in ("both", "right"):
                targets.update(rest_right)
            _imitate_hands_state["no_hand_sent"] = True
    else:
        _imitate_hands_state["no_hand_sent"] = False
        for det in valid_hands:
            handedness = _handedness(det)
            is_person_right = handedness > 0.5
            robot_side = "left" if is_person_right else "right"
            if side not in ("both", robot_side):
                continue

            pts = [
                (
                    det.keypoint_x[k] / fw,
                    det.keypoint_y[k] / fw,
                    det.keypoint_z[k],
                )
                for k in range(21)
            ]
            wx = (det.keypoint_x[0] / fw) * 1152.0
            wy = (det.keypoint_y[0] / fh) * 648.0
            a_thumb = _bend_angle(pts, 1, 2, 4, 3)
            a_idx = _bend_angle(pts, 5, 6, 8, 7)
            a_mid = _bend_angle(pts, 9, 10, 12, 11)
            a_rng = _bend_angle(pts, 13, 14, 16, 15)
            a_ltl = _bend_angle(pts, 17, 18, 20, 19)

            if robot_side == "right":
                if parts in ("all", "arms"):
                    sh_h = wx * 13.0 - 10000.0
                    sh_v = 5000.0 - wy * 13.0 - 2000.0
                    targets["upper_arm_right_rotation"] = -0.5 * sh_h
                    targets["shoulder_vertical_right"] = 0.5 * sh_v - 1000.0
                    targets["lower_arm_right_rotation"] = -7400.0
                    targets["elbow_right"] = 3700.0
                if parts in ("all", "fingers"):
                    targets["thumb_right_opposition"] = -3.0 * (
                        a_thumb * 5000.0 - 9000.0
                    )
                    targets["index_right_stretch"] = -(a_idx * 5000.0 - 6000.0)
                    targets["middle_right_stretch"] = -(a_mid * 5000.0 - 9000.0)
                    targets["ring_right_stretch"] = -(a_rng * 5000.0 - 9000.0)
                    targets["pinky_right_stretch"] = -(a_ltl * 5000.0 - 9000.0)
            else:
                if parts in ("all", "arms"):
                    sh_h = wx * 13.0 - 4000.0
                    sh_v = wy * 13.0 - 4000.0
                    targets["upper_arm_left_rotation"] = 0.5 * sh_h
                    targets["shoulder_vertical_left"] = -0.5 * sh_v - 1000.0
                    targets["elbow_left"] = 2500.0
                    targets["lower_arm_left_rotation"] = 9000.0
                if parts in ("all", "fingers"):
                    targets["thumb_left_opposition"] = -3.0 * (
                        a_thumb * 5000.0 - 9000.0
                    )
                    targets["index_left_stretch"] = -(a_idx * 5000.0 - 9000.0)
                    targets["middle_left_stretch"] = -(a_mid * 5000.0 - 9000.0)
                    targets["ring_left_stretch"] = -(a_rng * 5000.0 - 6000.0)
                    targets["pinky_left_stretch"] = -(a_ltl * 5000.0 - 6000.0)

    clamped = {name: _clamp_joint(name, val) for name, val in targets.items()}
    deadband = 150.0
    names = []
    cdegs = []
    for name, cdeg in clamped.items():
        last = _imitate_hands_state["last_cdeg"].get(name)
        if last is None or abs(cdeg - last) >= deadband:
            names.append(name)
            cdegs.append(cdeg)
            _imitate_hands_state["last_cdeg"][name] = cdeg

    if not names:
        return

    request = ApplyJointTrajectory.Request()
    jt = JointTrajectory()
    jt.joint_names = names
    point = JointTrajectoryPoint()
    point.positions = [cdeg * math.pi / 18000.0 for cdeg in cdegs]
    jt.points = [point]
    request.joint_trajectory = jt
    _imitate_hands_state["future"] = apply_joint_trajectory_client.call_async(request)
`;

// set-solid-state-relay

export const SET_SOLID_STATE_RELAY_FUNCTION = (
  generator: typeof pythonGenerator,
) => `

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

// get-solid-state-relay

export const GET_SOLID_STATE_RELAY_FUNCTION = (
  generator: typeof pythonGenerator,
) => `

def ${generator.FUNCTION_NAME_PLACEHOLDER_}() -> bool:

    received = {}

    def _on_relay_state(msg):
        received["turned_on"] = msg.turned_on

    subscription = node.create_subscription(
        SolidStateRelayState, "solid_state_relay_state", _on_relay_state, 10
    )

    logging.info(f"waiting for solid state relay state...")
    while "turned_on" not in received:
        rclpy.spin_once(node)
    node.destroy_subscription(subscription)

    logging.info(f"solid state relay is {'ON' if received['turned_on'] else 'OFF'}.")
    return received["turned_on"]
`;

// get-sound-direction (DOA)

export const GET_SOUND_DIRECTION_FUNCTION = (
  generator: typeof pythonGenerator,
) => `

def ${generator.FUNCTION_NAME_PLACEHOLDER_}() -> int:

    received = {}
    timeout_sec = 5.0

    def _on_doa_angle(msg):
        received["data"] = msg.data

    subscription = node.create_subscription(
        Int32, "/doa_angle", _on_doa_angle, 10
    )

    logging.info("waiting for sound direction...")
    deadline = time.time() + timeout_sec
    while "data" not in received and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(subscription)

    if "data" not in received:
        logging.warning("no sound direction received within 5.0 s; returning 0.")
        return 0

    return int(received["data"])
`;

// run-script

export const RUN_SCRIPT_FUNCTION = (generator: typeof pythonGenerator) => `

def ${generator.FUNCTION_NAME_PLACEHOLDER_}(script: str, host: str, user: str, password: str, port: int) -> None:

    effective_host = host
    if host in ("localhost", "127.0.0.1"):
        effective_host = os.environ.get("SSH_HOST", host)

    logging.info(f"connecting to {user}@{effective_host}:{port} via ssh...")

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    try:
        client.connect(
            hostname=effective_host,
            port=port,
            username=user,
            password=password,
            allow_agent=False,
            look_for_keys=False,
            timeout=10,
        )

        logging.info(f"running script on {effective_host}...")
        stdin, stdout, stderr = client.exec_command(script)
        exit_status = stdout.channel.recv_exit_status()

        out = stdout.read().decode()
        err = stderr.read().decode()

        if out:
            logging.info(out)
        if err:
            logging.error(err)

        logging.info(f"script finished with exit code {exit_status}.")
    except Exception as e:
        logging.error(f"Cannot connect to {effective_host} via ssh: {e}")
    finally:
        client.close()
`;

// vision

export const VISION_HELPER_CLASS = (generator: typeof pythonGenerator) => `
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

export const TF_BUTTON_TASTER_FUNCTION = (
  generator: typeof pythonGenerator,
) => `
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

export const TF_BUTTON_SWITCH_FUNCTION = (
  generator: typeof pythonGenerator,
) => `
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

export const TF_BUTTON_SET_COLOR_FUNCTION = (
  generator: typeof pythonGenerator,
) => `
def ${generator.FUNCTION_NAME_PLACEHOLDER_}(button_id: int, red: int, green: int, blue: int) -> None:
    logging.info(f"setting Tinkerforge button {button_id} color to rgb({red}, {green}, {blue}).")

    # Consolidated API: publish sticky color until overwritten.
    blockly_client.set_button_color(button_id, red, green, blue)
`;

export const PROGRAM_LOG_PATH_FUNCTION = (
  generator: typeof pythonGenerator,
) => `
def ${generator.FUNCTION_NAME_PLACEHOLDER_}() -> str:
    program_number = os.path.splitext(os.path.basename(__file__))[0]
    log_dir = os.path.join(os.getenv("PROGRAM_DIR", "/home/pib/cerebra_programs"), "program-logs")
    os.makedirs(log_dir, exist_ok=True)
    return os.path.join(log_dir, f"{program_number}.log")
`;

export const PROGRAM_LOG_FUNCTION = (
  generator: typeof pythonGenerator,
  pathFunctionName: string,
) => `
def ${generator.FUNCTION_NAME_PLACEHOLDER_}(level, text) -> None:
    with open(${pathFunctionName}(), "a", encoding="utf-8") as log_file:
        log_file.write(f"[{datetime.datetime.now()}] {level} {text}\\n")
`;

export const PROGRAM_RESET_LOG_FUNCTION = (
  generator: typeof pythonGenerator,
  pathFunctionName: string,
) => `
def ${generator.FUNCTION_NAME_PLACEHOLDER_}() -> None:
    with open(${pathFunctionName}(), "w", encoding="utf-8") as log_file:
        log_file.write("")
`;
