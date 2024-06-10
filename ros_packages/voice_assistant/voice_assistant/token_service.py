import os
import secrets
from base64 import urlsafe_b64encode
from hashlib import scrypt

import rclpy
from cryptography.fernet import Fernet
from datatypes.srv import EncryptToken, DecryptToken
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from . import SECRETS_DIR, SALT_PATH, TOKEN_PATH


class TokenServiceNode(Node):
    def __init__(self):
        super().__init__("token_service")

        self.get_logger().info("Now running Token Service")
        self.encryption_service = self.create_service(
            EncryptToken, "encrypt_token", self.encrypt_token_callback
        )
        self.decryption_service = self.create_service(
            DecryptToken, "decrypt_token", self.decrypt_token_callback
        )
        self.token_publisher = self.create_publisher(String, "public_api_token", 10)

    def encrypt_token_callback(
        self, request: EncryptToken.Request, response: EncryptToken.Response
    ):
        token: str = request.token
        password: str = request.password

        is_successful: bool = self.encrypt_token(token, password)

        response.is_successful = is_successful
        return response

    def decrypt_token_callback(
        self, request: DecryptToken.Request, response: DecryptToken.Response
    ):
        password: str = request.password
        is_successful: bool = True

        try:
            token: str = self.decrypt_token(password)

            # Publish decrypted Token
            msg = String()
            msg.data = token
            self.token_publisher.publish(msg)
        except Exception:
            self.get_logger().warn(f"token could not be decrypted")
            is_successful = False

        response.is_successful = is_successful
        return response

    def _generate_salt(self, size: int = 16) -> bytes:
        return secrets.token_bytes(size)

    def _generate_key(self, salt: bytes, password: str) -> bytes:
        encoded_password: bytes = str.encode(password)
        key = scrypt(encoded_password, salt=salt, n=2**14, r=8, p=1, dklen=32)
        return urlsafe_b64encode(key)

    def _store_secret(self, secret: bytes, filename: str) -> None:
        if not os.path.exists(SECRETS_DIR):
            os.makedirs(SECRETS_DIR)

        with open(filename, "wb") as file:
            file.write(secret)

    def _load_secret(self, filename: str) -> bytes:
        with open(filename, "rb") as file:
            data = file.read()
        return data

    def encrypt_token(self, token: str, password: str) -> bool:
        if len(password) < 8:
            return False

        try:
            encoded_token: bytes = str.encode(token)
            salt: bytes = self._generate_salt()
            key: bytes = self._generate_key(salt, password)
        except Exception as e:
            self.get_logger().error(f"Could not generate salt/key: {e}")
            return False

        try:
            f = Fernet(key)
            encrypted_token: bytes = f.encrypt(encoded_token)
        except Exception as e:
            self.get_logger().error(f"Could not encrypt token: {e}")
            return False

        try:
            self._store_secret(secret=salt, filename=SALT_PATH)
            self._store_secret(secret=encrypted_token, filename=TOKEN_PATH)
        except Exception as e:
            self.get_logger().error(f"Could not store secrets: {e}")
            return False

        return True

    def decrypt_token(self, password: str) -> str:
        salt: bytes = self._load_secret(SALT_PATH)
        encrypted_token: bytes = self._load_secret(TOKEN_PATH)

        key: bytes = self._generate_key(salt, password)
        f = Fernet(key)
        decrypted_token: bytes = f.decrypt(encrypted_token)
        return decrypted_token.decode("utf-8")


def main():
    rclpy.init()
    node = TokenServiceNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
