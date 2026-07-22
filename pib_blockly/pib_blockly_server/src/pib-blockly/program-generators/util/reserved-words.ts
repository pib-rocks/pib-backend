export const RESERVED_WORDS = [
    // common
    "rclpy",
    "node",
    "time",
    "sys",
    "np",
    "dai",
    "logging",
    "blobconverter",
    "stdout_handler",
    "stderr_handler",
    // play-audio-from-speech
    "PlayAudioFromSpeech",
    "play_audio_from_speech_client",
    // motor
    "ApplyJointTrajectory",
    "GetJointPosition",
    "JointTrajectory",
    "JointTrajectoryPoint",
    "apply_joint_trajectory_client",
    "get_joint_position_client",
    // face detector
    "fd",
    // pose
    "pose_client",
    // solid-state relay
    "SetSolidStateRelay",
    "set_solid_state_relay_state_client",
].join(",");
