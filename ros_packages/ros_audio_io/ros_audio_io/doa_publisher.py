"""Own XVF3000 telemetry, tuning and LEDs in ros-audio-io.

The parameters declared by this node are the microphone array control surface
for the UI through rosbridge. No Flask process should open the USB device.
"""

import os

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Int32
import usb.core
import usb.util

from ros_audio_io.device_retry import (
    DeviceNotFoundError,
    describe_open_failure,
    next_retry_delay,
)
from ros_audio_io.microphone_parameters import (
    LED_DEFAULTS,
    PARAMETER_SPECS,
    PRESETS,
    TUNABLE_PARAMETERS,
    apply_tuning_values,
    validate_parameter,
)
from ros_audio_io.pixel_ring import PixelRing
from ros_audio_io.tuning import Tuning


class MicrophoneArrayNode(Node):
    """Publish XVF3000 state at 10 Hz and provide its ROS parameter controls."""

    def __init__(self):
        super().__init__("microphone_array")
        self.doa_publisher = self.create_publisher(Int32, "/doa_angle", 10)
        self.voice_publisher = self.create_publisher(Bool, "/voice_activity", 10)
        self.speech_publisher = self.create_publisher(Bool, "/speech_detected", 10)
        self._read_warning_active = False
        self._syncing_parameters = False
        self._pending_parameter_sync = {}
        self._sync_timer = None
        self.retry_timer = None
        self.open_attempts = 0
        self.dev = None
        self.tuning = None
        self.pixel_ring = None

        self._attempt_device_open()

        self._declare_control_parameters()
        self.add_on_set_parameters_callback(self._on_parameters_changed)

    def _attempt_device_open(self):
        self.open_attempts += 1
        device = None
        try:
            device = usb.core.find(idVendor=0x2886, idProduct=0x0018)
            if device is None:
                raise DeviceNotFoundError("ReSpeaker Mic Array v2.0 was not found")
            tuning = Tuning(device)
            pixel_ring = PixelRing(device)
        except Exception as exc:
            if device is not None:
                try:
                    usb.util.dispose_resources(device)
                except Exception:
                    pass
            delay = next_retry_delay(self.open_attempts)
            reason = describe_open_failure(exc)
            self.get_logger().warning(
                f"Microphone array open failed: reason={reason}; detail={exc}; "
                f"attempt={self.open_attempts}; next attempt in {delay:g}s; "
                "publishing no device state"
            )
            self.retry_timer = self.create_timer(delay, self.retry_open_device)
            return

        self.dev = device
        self.tuning = tuning
        self.pixel_ring = pixel_ring
        publish_hz = self._publish_rate()
        self.get_logger().info(
            f"Publishing DOA, voice activity and speech detection at {publish_hz:g} Hz"
        )
        self.timer = self.create_timer(1.0 / publish_hz, self.publish_state)

    def retry_open_device(self):
        retry_timer = self.retry_timer
        self.retry_timer = None
        if retry_timer is not None:
            retry_timer.cancel()
            self.destroy_timer(retry_timer)
        self._attempt_device_open()

    def _publish_rate(self):
        try:
            publish_hz = float(os.getenv("MIC_ARRAY_PUBLISH_HZ", "10"))
            if publish_hz <= 0:
                raise ValueError
            return publish_hz
        except ValueError:
            self.get_logger().warning(
                "Invalid MIC_ARRAY_PUBLISH_HZ; publishing device state at 10 Hz"
            )
            return 10.0

    def _declare_control_parameters(self):
        """Declare values, using register reads when the device is available."""

        self.declare_parameter("preset", "Custom")
        for name in TUNABLE_PARAMETERS:
            value = PARAMETER_SPECS[name][3]
            if self.tuning is not None:
                try:
                    value = self.tuning.read(name)
                except Exception as exc:
                    self.get_logger().warning(
                        f"Could not read initial {name}; using declared default: {exc}"
                    )
            self.declare_parameter(name, value)
        for name, value in LED_DEFAULTS.items():
            self.declare_parameter(name, value)

    def _on_parameters_changed(self, parameters):
        if self._syncing_parameters:
            return SetParametersResult(successful=True)

        try:
            updates = {
                parameter.name: validate_parameter(parameter.name, parameter.value)
                for parameter in parameters
            }
        except ValueError as exc:
            return SetParametersResult(successful=False, reason=str(exc))

        if self.tuning is None or self.pixel_ring is None:
            return SetParametersResult(
                successful=False,
                reason="ReSpeaker device is unavailable; no value was applied",
            )

        tuning_updates = {}
        preset = updates.get("preset")
        if preset is not None and preset != "Custom":
            tuning_updates.update(PRESETS[preset])
        tuning_updates.update(
            {
                name: value
                for name, value in updates.items()
                if name in TUNABLE_PARAMETERS
            }
        )

        readbacks, failure = apply_tuning_values(self.tuning, tuning_updates)
        if failure is None:
            try:
                led_names = set(LED_DEFAULTS).intersection(updates)
                if led_names:
                    led_state = {
                        name: self.get_parameter(name).value for name in LED_DEFAULTS
                    }
                    led_state.update({name: updates[name] for name in led_names})
                    self._apply_led_state(led_state)
            except Exception as exc:
                failure = f"LED update failed: {exc}"

        if failure is not None:
            if readbacks:
                self._schedule_parameter_sync({**readbacks, "preset": "Custom"})
            self.get_logger().error(failure)
            return SetParametersResult(successful=False, reason=failure)

        if preset is not None and preset != "Custom":
            self._schedule_parameter_sync(readbacks)
        elif tuning_updates:
            self._schedule_parameter_sync({"preset": "Custom"})
        return SetParametersResult(successful=True)

    def _apply_led_state(self, state):
        """Apply LED state; the firmware acknowledges writes but cannot read back."""

        self.pixel_ring.set_brightness(state["led_brightness"])
        self.pixel_ring.set_vad_led(state["vad_led"])
        mode = state["led_mode"]
        if mode == "mono":
            self.pixel_ring.mono(int(state["led_color"][1:], 16))
        else:
            getattr(self.pixel_ring, mode)()

    def _schedule_parameter_sync(self, values):
        """Reflect device read-backs without recursively writing the device."""

        if not values:
            return
        self._pending_parameter_sync.update(values)
        if self._sync_timer is None:
            self._sync_timer = self.create_timer(0.01, self._sync_parameters)

    def _sync_parameters(self):
        self._sync_timer.cancel()
        self.destroy_timer(self._sync_timer)
        self._sync_timer = None
        values = self._pending_parameter_sync
        self._pending_parameter_sync = {}
        self._syncing_parameters = True
        try:
            results = self.set_parameters(
                [Parameter(name=name, value=value) for name, value in values.items()]
            )
            if any(not result.successful for result in results):
                self.get_logger().error("Could not reflect device parameter read-back")
        finally:
            self._syncing_parameters = False

    def publish_state(self):
        try:
            direction = int(self.tuning.read("DOAANGLE"))
            voice_activity = bool(self.tuning.read("VOICEACTIVITY"))
            speech_detected = bool(self.tuning.read("SPEECHDETECTED"))
        except Exception as exc:
            if not self._read_warning_active:
                self.get_logger().warning(
                    f"XVF3000 state read failed; publishing nothing: {exc}"
                )
                self._read_warning_active = True
            return

        self._read_warning_active = False
        self.doa_publisher.publish(Int32(data=direction))
        self.voice_publisher.publish(Bool(data=voice_activity))
        self.speech_publisher.publish(Bool(data=speech_detected))

    def destroy_node(self):
        if self.dev is not None:
            usb.util.dispose_resources(self.dev)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MicrophoneArrayNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
