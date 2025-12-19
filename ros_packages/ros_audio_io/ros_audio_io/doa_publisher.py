#!/usr/bin/env python3
"""
DOA publisher for ReSpeaker Mic Array v2.0 optimized for low CPU usage.

- Polls the device in a background thread with sleeping (no busy loop).
- Publishes only when the direction (DOA) changes.
- Attempts automatic reconnection if device is unplugged.
- Minimal logging in the hot path (use DEBUG to see publishes).
"""

import os
import time
import threading
from typing import Optional

import usb.core
import usb.util
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

from ros_audio_io.tuning import Tuning


class LowCpuDOAPublisher(Node):
    VENDOR_ID = 0x2886
    PRODUCT_ID = 0x0018
    DEFAULT_DOA_PUBLISH_INTERVAL = 0.2
    DEFAULT_RECONNECT_INTERVAL = 5.0

    def __init__(self):
        super().__init__("doa_publisher")

        # Publisher
        self.publisher_ = self.create_publisher(Int32, "doa_angle", 10)

        # Configurable intervals
        self.poll_interval = self._get_env_float(
            "DOA_PUBLISH_INTERVAL", self.DEFAULT_DOA_PUBLISH_INTERVAL
        )
        self.reconnect_interval = self._get_env_float(
            "DOA_RECONNECT_INTERVAL", self.DEFAULT_RECONNECT_INTERVAL
        )

        # USB device / tuning handle
        self._dev = None  # type: Optional[usb.core.Device]
        self._tuning = None  # type: Optional[Tuning]
        self._last_direction = None  # last published direction

        # Thread control
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._worker_loop, daemon=True)

        # Start worker thread (it handles connection attempts internally)
        self.get_logger().info(
            f"DOA poll interval: {self.poll_interval}s, reconnect interval: {self.reconnect_interval}s"
        )
        self._thread.start()

    def _get_env_float(self, varname: str, default: float) -> float:
        val = os.getenv(varname)
        if val is None:
            return default
        try:
            return float(val)
        except ValueError:
            self.get_logger().warning(
                f"Invalid {varname}='{val}', using default {default}"
            )
            return default

    def _find_device(self) -> Optional[usb.core.Device]:
        """Try to find the ReSpeaker device by vendor/product id."""
        try:
            dev = usb.core.find(idVendor=self.VENDOR_ID, idProduct=self.PRODUCT_ID)
            return dev
        except Exception as e:
            self.get_logger().debug(f"USB find exception: {e}")
            return None

    def _open_device(self) -> bool:
        """Attempt to open the device and create the Tuning wrapper."""
        dev = self._find_device()
        if dev is None:
            return False

        try:
            # If device was previously claimed, release resources first
            usb.util.dispose_resources(dev)
        except Exception:
            # safe to ignore; means nothing to dispose
            pass

        try:
            # Instantiate Tuning (this may perform USB control transfers)
            tuning = Tuning(dev)
            # store only if successful
            self._dev = dev
            self._tuning = tuning
            self.get_logger().info("Connected to ReSpeaker Mic Array v2.0")
            return True
        except Exception as e:
            # Could not initialize the tuning wrapper
            self.get_logger().warning(f"Failed to init Tuning: {e}")
            self._dev = None
            self._tuning = None
            return False

    def _close_device(self):
        """Clean up usb resources when device is lost or on shutdown."""
        if self._dev is not None:
            try:
                usb.util.dispose_resources(self._dev)
            except Exception:
                pass
        self._dev = None
        self._tuning = None
        self.get_logger().info("ReSpeaker disconnected")

    def _worker_loop(self):
        """
        Background worker:
        - tries to maintain a connection to the device
        - polls the DOA at poll_interval when connected
        - on disconnect, sleeps reconnect_interval and tries again
        """
        while not self._stop_event.is_set():
            if self._tuning is None:
                # Try to (re)connect
                if self._open_device():
                    # reset last_direction so we publish the first valid reading
                    self._last_direction = None
                else:
                    # wait and try again
                    if self._stop_event.wait(self.reconnect_interval):
                        break
                    continue

            # At this point we have self._tuning
            try:
                # Read direction from the Tuning wrapper
                direction = self._tuning.direction

                # Only publish when it changes (cheap integer compare)
                if direction != self._last_direction:
                    self._last_direction = direction
                    msg = Int32(data=int(direction))
                    # Publishing from a background thread is okay in rclpy
                    self.publisher_.publish(msg)
                    self.get_logger().debug(f"Published DOA angle: {direction}")
                    self.get_logger().info(f"Published DOA angle: {direction}")

                # Sleep for poll interval (interruptible by stop_event)
                if self._stop_event.wait(self.poll_interval):
                    break

            except usb.core.USBError as e:
                # Device likely unplugged or transient USB error occurred
                self.get_logger().warning(f"USB error while reading DOA: {e}")
                self._close_device()
                # try to reconnect after a short delay
                if self._stop_event.wait(self.reconnect_interval):
                    break
            except Exception as e:
                # Catch-all for unexpected errors from Tuning
                self.get_logger().error(f"Unexpected error while reading DOA: {e}")
                # avoid tight error loop: wait a bit before trying again
                if self._stop_event.wait(min(self.reconnect_interval, 1.0)):
                    break

    def destroy_node(self):
        """Stop the worker thread cleanly then destroy the node."""
        self.get_logger().debug("Shutting down DOA worker thread...")
        self._stop_event.set()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        # close usb resources if any
        self._close_device()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LowCpuDOAPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down DOA node...")
    finally:
        # ensure node and background thread are stopped
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
