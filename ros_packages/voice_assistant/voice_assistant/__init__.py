import os


VOICE_ASSISTANT_DIRECTORY = os.getenv("VOICE_ASSISTANT_DIR", "~/app")

# Tryb Token Encryption
SALT_FILENAME = "salt.salt"
ENCRYPTED_TOKEN_FILENAME = "token.aes"
SECRETS_DIR = f"{VOICE_ASSISTANT_DIRECTORY}/secrets"

SALT_PATH = f"{SECRETS_DIR}/{SALT_FILENAME}"
TOKEN_PATH = f"{SECRETS_DIR}/{ENCRYPTED_TOKEN_FILENAME}"
