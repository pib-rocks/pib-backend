import os

VOICE_ASSISTANT_DIRECTORY = os.getenv(
    "VOICE_ASSISTANT_DIR", "/home/pib/ros_working_dir/src/voice_assistant"
)

# Voice Assistant
START_SIGNAL_FILE = (
    f"{VOICE_ASSISTANT_DIRECTORY}/audiofiles/assistant_start_listening.wav"
)
STOP_SIGNAL_FILE = (
    f"{VOICE_ASSISTANT_DIRECTORY}/audiofiles/assistant_stop_listening.wav"
)
