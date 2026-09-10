export const RESERVED_WORDS = [
    // common
    "rclpy",
    "node",
    "time",
    "sys",
    "np",
    "dai",
    "logging",
    "datetime",
    "program_log_path",
    "program_log",
    "program_reset_log",
    "blobconverter",
    "stdout_handler",
    "stderr_handler",
    // play-audio-from-speech
    "PlayAudioFromSpeech",
    "play_audio_from_speech_client",
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
    // solid-state relay
    "SetSolidStateRelay",
    "set_solid_state_relay_state_client",
].join(",");
