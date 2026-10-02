import os
from typing import Optional

import rclpy
from datatypes.srv import EncryptToken, DecryptToken, GetTokenExists
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String, Empty

from pib_hermes_config import token_crypto

from . import SECRETS_DIR, SALT_PATH, TOKEN_PATH


class TokenServiceNode(Node):
    def __init__(self):
        super().__init__("token_service")
        self.active_token: Optional[str] = None
        self.is_token_stored: bool = self._check_if_previous_token_stored()

        self.delete_token_subscription = self.create_subscription(
            Empty, "delete_token", self.delete_token_callback, 10
        )
        self.get_token_exists_service = self.create_service(
            GetTokenExists, "get_token_exists", self.get_token_exists_callback
        )
        self.encryption_service = self.create_service(
            EncryptToken, "encrypt_token", self.encrypt_token_callback
        )
        self.decryption_service = self.create_service(
            DecryptToken, "decrypt_token", self.decrypt_token_callback
        )
        self.token_publisher = self.create_publisher(String, "public_api_token", 10)

        self.get_logger().info("Now Running TOKEN SERVICE")

    def publish_token(self, token: str) -> None:
        self.active_token = token
        msg = String()
        msg.data = token
        self.token_publisher.publish(msg)

    def delete_token_callback(self, _: Empty) -> None:
        try:
            if self._check_if_previous_token_stored():
                os.remove(TOKEN_PATH)
                os.remove(SALT_PATH)
        except Exception as e:
            self.get_logger().warn(f"Token file could not be deleted: {e}")

        # even if token stored in file cannot be deleted, remove the active token
        self.is_token_stored = False
        self.publish_token("")

    def get_token_exists_callback(
        self, _: GetTokenExists.Request, response: GetTokenExists.Response
    ):
        response.token_exists = self.is_token_stored
        if self.active_token:
            response.token_active = True
        return response

    def encrypt_token_callback(
        self, request: EncryptToken.Request, response: EncryptToken.Response
    ):
        token: str = request.token
        password: str = request.password

        is_successful: bool = self.encrypt_token(token, password)
        if is_successful:
            self.publish_token(token)

        self.is_token_stored = is_successful
        response.successful = is_successful
        return response

    def decrypt_token_callback(
        self, request: DecryptToken.Request, response: DecryptToken.Response
    ):
        password: str = request.password
        is_successful = True

        try:
            token: str = self.decrypt_token(password)
            self.publish_token(token)
        except Exception:
            self.get_logger().warn(f"Token could not be decrypted")
            is_successful = False

        response.successful = is_successful
        return response

    def _check_if_previous_token_stored(self) -> bool:
        return os.path.isfile(TOKEN_PATH)

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
        encrypted = token_crypto.encrypt_token(token, password)
        if encrypted is None:
            if len(password) >= token_crypto.MIN_PASSWORD_LENGTH:
                self.get_logger().error("Could not encrypt token")
            return False

        salt, ciphertext = encrypted
        try:
            self._store_secret(secret=salt, filename=SALT_PATH)
            self._store_secret(secret=ciphertext, filename=TOKEN_PATH)
        except Exception as e:
            self.get_logger().error(f"Could not store secrets: {e}")
            return False

        return True

    def decrypt_token(self, password: str) -> str:
        salt: bytes = self._load_secret(SALT_PATH)
        encrypted_token: bytes = self._load_secret(TOKEN_PATH)
        return token_crypto.decrypt_token(password, salt, encrypted_token)


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
