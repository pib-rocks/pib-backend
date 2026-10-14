from typing import Optional

import rclpy
from datatypes.srv import DecryptToken, EncryptToken, GetTokenExists
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Empty, String

from .cloud_token import (
    CloudTokenError,
    cloud_token_is_stored,
    delete_stored_cloud_token,
    log_cloud_token_source,
    read_cloud_token,
    submit_cloud_token,
)


class TokenServiceNode(Node):
    def __init__(self):
        super().__init__("token_service")
        self.active_token: Optional[str] = None
        self.is_token_stored: bool = False
        self._source_announced = False
        self._locked_announced = False

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
        self.create_timer(2.0, self._refresh_from_store)

        self.get_logger().info("Now Running TOKEN SERVICE")
        self._refresh_from_store()

    def publish_token(self, token: str) -> None:
        self.active_token = token or None
        msg = String()
        msg.data = token
        self.token_publisher.publish(msg)

    def delete_token_callback(self, _: Empty) -> None:
        try:
            delete_stored_cloud_token()
        except CloudTokenError as error:
            self.get_logger().warn("%s", error)
            return
        self.is_token_stored = False
        self._source_announced = False
        self.publish_token("")

    def get_token_exists_callback(
        self, _: GetTokenExists.Request, response: GetTokenExists.Response
    ):
        self._refresh_from_store()
        response.token_exists = self.is_token_stored
        response.token_active = bool(self.active_token)
        return response

    def encrypt_token_callback(
        self, request: EncryptToken.Request, response: EncryptToken.Response
    ):
        # The service definition still has a password field. It is not read.
        try:
            submit_cloud_token(request.token)
        except CloudTokenError as error:
            self.get_logger().warn("%s", error)
            response.successful = False
            return response
        self._refresh_from_store()
        response.successful = True
        return response

    def decrypt_token_callback(
        self, request: DecryptToken.Request, response: DecryptToken.Response
    ):
        # Activation is the key store unlock. This call does not take a password.
        del request
        self._refresh_from_store()
        response.successful = bool(self.active_token)
        return response

    def _refresh_from_store(self) -> None:
        try:
            token = read_cloud_token()
        except CloudTokenError as error:
            self._source_announced = False
            if not self._locked_announced:
                self.get_logger().warn("%s", error)
                self._locked_announced = True
            if self.active_token:
                self.publish_token("")
            self.is_token_stored = cloud_token_is_stored()
            return
        except Exception:
            self.get_logger().warn("Cloud token could not be read.")
            return

        self._locked_announced = False
        self.is_token_stored = True
        if token == self.active_token and self._source_announced:
            return
        if not self._source_announced:
            log_cloud_token_source()
            self._source_announced = True
        if token != self.active_token:
            self.publish_token(token)


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
