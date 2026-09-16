export const RESERVED_WORDS = [
    // common
    "rclpy",
    "node",
    "time",
    "sys",
    "np",
    "logging",
    "signal",
    "datetime",
    "program_log_path",
    "program_log",
    "program_reset_log",
    "stdout_handler",
    "stderr_handler",
    // play-audio-from-speech
    "PlayAudioFromSpeech",
    "play_audio_from_speech_client",
    // set-volume
    "SetVolume",
    "set_volume_client",
    // motor
    "ik",
    "Write",
    "right_arm",
    "left_arm",
    "ApplyJointTrajectory",
    "GetJointPosition",
    "JointTrajectory",
    "JointTrajectoryPoint",
    "apply_joint_trajectory_client",
    "get_joint_position_client",
    "Camera",
    // face detector
    "fd",
    // pose
    "pose_client",
    "pose_backend",
    "play_pose_sequence_timed",
    // solid-state relay
    "SetSolidStateRelay",
    "set_solid_state_relay_state_client",
    // sound direction (DOA)
    "Int32",
    // model inference
    "StartModel",
    "StopModel",
    "DetectionArray",
    "_blockly_model_manager",
].join(",");
