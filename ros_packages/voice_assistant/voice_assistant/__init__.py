import os


VOICE_ASSISTANT_DIRECTORY = os.getenv(
    "VOICE_ASSISTANT_DIR", "/home/pib/ros_working_dir/src/voice_assistant"
)

# Tryb Token Encryption
SALT_FILENAME = "salt.salt"
ENCRYPTED_TOKEN_FILENAME = "token.aes"
SECRETS_DIR = f"{VOICE_ASSISTANT_DIRECTORY}/secrets"

SALT_PATH = f"{SECRETS_DIR}/{SALT_FILENAME}"
TOKEN_PATH = f"{SECRETS_DIR}/{ENCRYPTED_TOKEN_FILENAME}"

# Voice Assistant
START_SIGNAL_FILE = (
    f"{VOICE_ASSISTANT_DIRECTORY}/audiofiles/assistant_start_listening.wav"
)
STOP_SIGNAL_FILE = (
    f"{VOICE_ASSISTANT_DIRECTORY}/audiofiles/assistant_stop_listening.wav"
)
MAX_SILENT_SECONDS_BEFORE = 8.0
