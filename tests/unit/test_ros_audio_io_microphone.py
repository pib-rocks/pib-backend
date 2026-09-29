"""Plain-Python tests for ros-audio-io microphone logic (no ROS required)."""

from __future__ import annotations

import importlib
import json
import math
import sys
import types
from pathlib import Path
from urllib import error

import pytest

ROS_AUDIO_PACKAGE = Path(__file__).parents[2] / "ros_packages" / "ros_audio_io"
sys.path.insert(0, str(ROS_AUDIO_PACKAGE))

from ros_audio_io.levels import calculate_levels  # noqa: E402
from ros_audio_io.device_retry import (  # noqa: E402
    DeviceNotFoundError,
    describe_open_failure,
    device_status_payload,
    next_retry_delay,
)
from ros_audio_io.desired_state import (  # noqa: E402
    DEFAULT_FLASK_API_BASE_URL,
    DESIRED_STATE_PATH,
    DesiredStateValidationError,
    build_desired_state_url,
    describe_desired_state_failure,
    desired_state_failure,
    fetch_desired_state,
    should_apply_revision,
    validate_desired_state,
)
from ros_audio_io.microphone_parameters import (  # noqa: E402
    AGCTIME_BLOCK_RATE_HZ,
    AGCTIME_READBACK_EPSILON_SECONDS,
    DEFAULT_READBACK_TOLERANCE,
    PRESETS,
    TUNABLE_PARAMETERS,
    agctime_coefficient,
    agctime_seconds,
    apply_tuning_values,
    parameter_from_readback,
    readback_matches,
    readback_mismatch_reason,
    readback_tolerance,
    validate_parameter,
)

# Read-backs measured on 192.168.1.172 with a ReSpeaker Mic Array v2.0. They
# are coefficients of the 62.5 Hz block rate, not seconds.
MEASURED_AGCTIME_READBACKS = {
    0.5: 0.9685218567028642,
    1.0: 0.9841422392055392,
}
# Both measured read-backs sit this far above exp(-1 / (62.5 * seconds)).
MEASURED_COEFFICIENT_RESIDUAL = 1.5e-05


def desired_state_document(revision=1):
    return {
        "parameters": dict(PRESETS["Standard"]),
        "led_ring": {
            "mode": "off",
            "brightness": 16,
            "color": "#000000",
            "vad_led": 0,
        },
        "preset": "Standard",
        "updatedAt": "2026-09-22T07:50:00+00:00",
        "revision": revision,
    }


class CoefficientDevice:
    """Device double answering AGCTIME the way the XVF3000 was measured to.

    ``held_agctime_seconds`` models a device that stores a ramp time other than
    the requested one.
    """

    def __init__(self, held_agctime_seconds=None):
        self.written = {}
        self._held_agctime_seconds = held_agctime_seconds

    def write(self, name, value):
        self.written[name] = value

    def read(self, name):
        value = self.written[name]
        if name == "AGCTIME":
            if self._held_agctime_seconds is not None:
                value = self._held_agctime_seconds
            return agctime_coefficient(value) + MEASURED_COEFFICIENT_RESIDUAL
        return value


class SaturatedDevice(CoefficientDevice):
    """Device double whose AGCTIME register reads back outside (0, 1)."""

    def read(self, name):
        if name == "AGCTIME":
            return 1.0
        return super().read(name)


class BrokenDevice(CoefficientDevice):
    """Device double whose second write fails, as a USB timeout would."""

    def write(self, name, value):
        if self.written:
            raise OSError("usb timeout")
        super().write(name, value)


def test_retry_delay_is_monotonic_and_bounded():
    delays = [next_retry_delay(attempt) for attempt in range(1, 12)]

    assert delays[:6] == [1.0, 2.0, 4.0, 8.0, 16.0, 30.0]
    assert delays == sorted(delays)
    assert max(delays) == 30.0
    assert next_retry_delay(10_000) == 30.0


@pytest.mark.parametrize(
    ("error", "reason"),
    [
        (OSError(-9999, "Unanticipated host error"), "device busy"),
        (DeviceNotFoundError("no default input"), "no matching input device"),
        (RuntimeError("PortAudio failed"), "device open failure"),
    ],
)
def test_open_failure_wording(error, reason):
    assert describe_open_failure(error) == reason


def test_desired_state_url_uses_house_default_and_environment_override():
    assert build_desired_state_url({}) == (
        DEFAULT_FLASK_API_BASE_URL + DESIRED_STATE_PATH
    )
    assert build_desired_state_url(
        {"FLASK_API_BASE_URL": "http://backend.example/v1/"}
    ) == ("http://backend.example/v1" + DESIRED_STATE_PATH)


def test_fetch_parses_and_validates_desired_state():
    document = desired_state_document()

    class Response:
        def __enter__(self):
            return self

        def __exit__(self, *args):
            pass

        def read(self):
            return json.dumps(document).encode()

    calls = []

    def opener(url, timeout):
        calls.append((url, timeout))
        return Response()

    desired = fetch_desired_state("http://backend/desired", opener=opener)

    assert calls == [("http://backend/desired", 3.0)]
    assert desired["parameters"] == document["parameters"]
    assert desired["led_ring"]["vad_led"] is False
    assert desired["revision"] == 1


@pytest.mark.parametrize(
    ("mutate", "reason"),
    [
        (
            lambda document: document["parameters"].update({"UNKNOWN": 1}),
            "unknown parameter: UNKNOWN",
        ),
        (
            lambda document: document["parameters"].update({"AGCONOFF": "1"}),
            "AGCONOFF must be an integer",
        ),
        (
            lambda document: document.pop("led_ring"),
            "missing field: led_ring",
        ),
    ],
)
def test_invalid_desired_state_is_rejected_before_any_apply(mutate, reason):
    document = desired_state_document()
    mutate(document)
    device = CoefficientDevice()

    with pytest.raises(DesiredStateValidationError, match=reason):
        validated = validate_desired_state(document)
        apply_tuning_values(device, validated["parameters"])

    assert device.written == {}


def test_revision_decision_applies_only_a_new_revision():
    assert should_apply_revision(None, 4)
    assert not should_apply_revision(4, 4)
    assert should_apply_revision(4, 5)


def test_backend_unreachable_has_named_reason_and_bounded_backoff():
    failure = error.URLError("connection refused")

    assert describe_desired_state_failure(failure) == "backend unreachable"
    assert desired_state_failure(failure, 1) == ("backend unreachable", 1.0)
    assert desired_state_failure(failure, 100) == ("backend unreachable", 30.0)


def test_unavailable_status_has_no_invented_device_facts():
    payload = device_status_payload(
        available=False,
        reason="device busy",
        detail="[Errno -9999] Unanticipated host error",
        attempts=3,
        next_retry_in_seconds=8.0,
    )

    assert payload == {
        "available": False,
        "reason": "device busy",
        "detail": "[Errno -9999] Unanticipated host error",
        "attempts": 3,
        "nextRetryInSeconds": 8.0,
        "owner": "ros-audio-io",
    }
    assert not {"deviceName", "channels", "rate", "processedChannel"} & payload.keys()


def test_audio_streamer_survives_open_failure_and_recovers(monkeypatch):
    class FakeMessage:
        def __init__(self):
            self.data = None

    class FakePublisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    class FakeTimer:
        def __init__(self, delay, callback):
            self.delay = delay
            self.callback = callback
            self.cancelled = False

        def cancel(self):
            self.cancelled = True

    class FakeLogger:
        def info(self, message):
            pass

        def warning(self, message):
            pass

        def error(self, message):
            pass

    class FakeNode:
        def __init__(self, name):
            self.publishers = {}
            self.timers = []
            self.logger = FakeLogger()

        def create_publisher(self, message_type, topic, depth):
            publisher = FakePublisher()
            self.publishers[topic] = publisher
            return publisher

        def create_service(self, service_type, name, callback):
            return object()

        def create_timer(self, delay, callback):
            timer = FakeTimer(delay, callback)
            self.timers.append(timer)
            return timer

        def destroy_timer(self, timer):
            self.timers.remove(timer)

        def get_logger(self):
            return self.logger

        def destroy_node(self):
            pass

    class FakeStream:
        def stop_stream(self):
            pass

        def close(self):
            pass

    class FakePyAudio:
        def __init__(self):
            self.can_open = False

        def get_host_api_count(self):
            return 1

        def get_host_api_info_by_index(self, index):
            return {"index": index, "name": "ALSA"}

        def get_device_count(self):
            return 1

        def get_device_info_by_index(self, index):
            return {
                "name": "ReSpeaker 4 Mic Array (UAC1.0): USB Audio (hw:2,0)",
                "maxInputChannels": 6,
                "defaultSampleRate": 16000,
                "hostApi": 0,
            }

        def open(self, **kwargs):
            if not self.can_open:
                raise OSError(-9999, "Unanticipated host error")
            return FakeStream()

        def terminate(self):
            pass

    fake_audio = FakePyAudio()
    pyaudio = types.ModuleType("pyaudio")
    pyaudio.paInt16 = 8
    pyaudio.PyAudio = lambda: fake_audio
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda args=None: None
    rclpy.spin = lambda node: None
    rclpy.shutdown = lambda: None
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = FakeNode
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Float32MultiArray = FakeMessage
    std_msgs_msg.Int16MultiArray = FakeMessage
    std_msgs_msg.String = FakeMessage
    datatypes = types.ModuleType("datatypes")
    datatypes_srv = types.ModuleType("datatypes.srv")
    datatypes_srv.GetMicConfiguration = object

    for name, module in {
        "pyaudio": pyaudio,
        "rclpy": rclpy,
        "rclpy.node": rclpy_node,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
        "datatypes": datatypes,
        "datatypes.srv": datatypes_srv,
    }.items():
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.setenv("MIC_DEVICE", "respeaker")
    monkeypatch.delitem(sys.modules, "ros_audio_io.audio_streamer", raising=False)

    audio_streamer = importlib.import_module("ros_audio_io.audio_streamer")
    node = audio_streamer.AudioStreamer()

    unavailable = json.loads(node.status_pub.messages[-1].data)
    assert node.audio_stream is None
    assert node.retry_timer.delay == 1.0
    assert unavailable["available"] is False
    assert unavailable["reason"] == "device busy"
    assert "deviceName" not in unavailable

    fake_audio.can_open = True
    node.retry_open_device()

    available = json.loads(node.status_pub.messages[-1].data)
    assert node.audio_stream is not None
    assert available == {
        "available": True,
        "deviceName": "ReSpeaker 4 Mic Array (UAC1.0): USB Audio (hw:2,0)",
        "channels": 6,
        "rate": 16000,
        "processedChannel": 0,
        "owner": "ros-audio-io",
    }
    sys.modules.pop("ros_audio_io.audio_streamer", None)


def test_doa_publisher_retries_until_array_is_available(monkeypatch):
    class FakeTimer:
        def __init__(self, delay, callback):
            self.delay = delay
            self.callback = callback

        def cancel(self):
            pass

    class FakeLogger:
        def __init__(self):
            self.infos = []
            self.warnings = []

        def info(self, message):
            self.infos.append(message)

        def warning(self, message):
            self.warnings.append(message)

        def error(self, message):
            pass

    class FakeNode:
        def __init__(self, name):
            self.timers = []
            self.logger = FakeLogger()

        def create_publisher(self, message_type, topic, depth):
            return types.SimpleNamespace(publish=lambda message: None)

        def create_timer(self, delay, callback):
            timer = FakeTimer(delay, callback)
            self.timers.append(timer)
            return timer

        def destroy_timer(self, timer):
            self.timers.remove(timer)

        def declare_parameter(self, name, value):
            pass

        def add_on_set_parameters_callback(self, callback):
            pass

        def get_logger(self):
            return self.logger

    device_state = {"available": False}
    device = object()
    usb_core = types.ModuleType("usb.core")
    usb_core.find = lambda **kwargs: device if device_state["available"] else None
    usb_util = types.ModuleType("usb.util")
    usb_util.dispose_resources = lambda found: None
    usb = types.ModuleType("usb")
    usb.core = usb_core
    usb.util = usb_util

    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = FakeNode
    rclpy_parameter = types.ModuleType("rclpy.parameter")
    rclpy_parameter.Parameter = object
    rcl_interfaces = types.ModuleType("rcl_interfaces")
    rcl_interfaces_msg = types.ModuleType("rcl_interfaces.msg")
    rcl_interfaces_msg.SetParametersResult = object
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Bool = object
    std_msgs_msg.Int32 = object
    tuning = CoefficientDevice()
    led_calls = []
    tuning_module = types.ModuleType("ros_audio_io.tuning")
    tuning_module.Tuning = lambda found: tuning
    pixel_ring_module = types.ModuleType("ros_audio_io.pixel_ring")
    pixel_ring_module.PixelRing = lambda found: types.SimpleNamespace(
        set_brightness=lambda value: led_calls.append(("brightness", value)),
        set_vad_led=lambda value: led_calls.append(("vad_led", value)),
        off=lambda: led_calls.append(("mode", "off")),
    )

    for name, module in {
        "usb": usb,
        "usb.core": usb_core,
        "usb.util": usb_util,
        "rclpy": rclpy,
        "rclpy.node": rclpy_node,
        "rclpy.parameter": rclpy_parameter,
        "rcl_interfaces": rcl_interfaces,
        "rcl_interfaces.msg": rcl_interfaces_msg,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
        "ros_audio_io.tuning": tuning_module,
        "ros_audio_io.pixel_ring": pixel_ring_module,
    }.items():
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.delitem(sys.modules, "ros_audio_io.doa_publisher", raising=False)

    doa_publisher = importlib.import_module("ros_audio_io.doa_publisher")
    monkeypatch.setattr(
        doa_publisher,
        "fetch_desired_state",
        lambda url: validate_desired_state(desired_state_document(revision=7)),
    )
    node = doa_publisher.MicrophoneArrayNode()

    assert node.dev is None
    assert node.retry_timer.delay == 1.0

    device_state["available"] = True
    node.retry_open_device()

    assert node.dev is device
    assert node.tuning is not None
    assert node.pixel_ring is not None
    assert node.retry_timer is None
    assert node.timer.delay == pytest.approx(0.1)
    assert node.applied_desired_state_revision == 7
    assert set(tuning.written) == set(TUNABLE_PARAMETERS)
    assert led_calls == [
        ("brightness", 16),
        ("vad_led", False),
        ("mode", "off"),
    ]
    assert any(
        "Applied microphone desired state: revision=7" in message
        for message in node.logger.infos
    )

    writes_after_recovery = dict(tuning.written)
    node._periodic_reconcile()
    assert tuning.written == writes_after_recovery
    assert (
        len(
            [
                message
                for message in node.logger.infos
                if "Applied microphone desired state" in message
            ]
        )
        == 1
    )
    sys.modules.pop("ros_audio_io.doa_publisher", None)


def test_levels_silence_floor():
    assert calculate_levels([0] * 1024) == (0.0, 0.0)


def test_levels_full_scale():
    assert calculate_levels([-32768, 32767]) == pytest.approx(
        (math.sqrt((32768**2 + 32767**2) / 2) / 32768, 1.0)
    )


def test_levels_synthetic_sine():
    amplitude = 12000
    samples = [
        round(amplitude * math.sin(2 * math.pi * index / 128)) for index in range(128)
    ]
    rms, peak = calculate_levels(samples)
    assert rms == pytest.approx(amplitude / math.sqrt(2) / 32768, abs=1e-5)
    assert peak == pytest.approx(amplitude / 32768)


@pytest.mark.parametrize(
    ("name", "value"),
    [
        ("AGCONOFF", 0),
        ("AGCMAXGAIN", 1000.0),
        ("AGCDESIREDLEVEL", 1e-08),
        ("AGCTIME", 0.1),
        ("HPFONOFF", 3),
        ("preset", "Noisy Environment / ASR"),
        ("led_mode", "mono"),
        ("led_brightness", 31),
        ("led_color", "#12ABEF"),
        ("vad_led", True),
    ],
)
def test_parameter_validation_accepts_documented_values(name, value):
    assert validate_parameter(name, value) == value


@pytest.mark.parametrize(
    ("name", "value"),
    [
        ("AGCONOFF", 2),
        ("AGCMAXGAIN", 0.5),
        ("AGCMAXGAIN", float("nan")),
        ("AGCTIME", 1.1),
        ("HPFONOFF", 4),
        ("preset", "Unknown"),
        ("led_mode", "rainbow"),
        ("led_brightness", 32),
        ("led_color", "12ABEF"),
        ("vad_led", 1),
    ],
)
def test_parameter_validation_rejects_invalid_values(name, value):
    with pytest.raises(ValueError):
        validate_parameter(name, value)


def test_presets_contain_every_tuning_parameter():
    standard_names = set(PRESETS["Standard"])
    for name, values in PRESETS.items():
        if name != "Custom":
            assert set(values) == standard_names


def test_float_readback_allows_xvf_fixed_point_rounding():
    assert readback_matches("AGCDESIREDLEVEL", 0.005, 0.00500001)
    assert not readback_matches("AGCDESIREDLEVEL", 0.005, 0.006)


def test_noisy_preset_keeps_its_intended_half_second_ramp():
    assert PRESETS["Noisy Environment / ASR"]["AGCTIME"] == 0.5


def test_block_rate_is_the_processing_rate_of_the_device():
    assert AGCTIME_BLOCK_RATE_HZ == 16000 / 256 == 62.5


@pytest.mark.parametrize("requested", sorted(MEASURED_AGCTIME_READBACKS))
def test_measured_readbacks_follow_the_block_rate_model(requested):
    measured = MEASURED_AGCTIME_READBACKS[requested]

    # 1.49e-05 at 1.0 s and 1.53e-05 at 0.5 s: one systematic residual.
    assert measured - agctime_coefficient(requested) == pytest.approx(
        MEASURED_COEFFICIENT_RESIDUAL, abs=5e-07
    )
    assert agctime_seconds(measured) == pytest.approx(
        requested, abs=AGCTIME_READBACK_EPSILON_SECONDS
    )


@pytest.mark.parametrize("seconds", [0.1, 0.25, 0.5, 1.0])
def test_agctime_converts_from_seconds_and_back(seconds):
    coefficient = agctime_coefficient(seconds)

    assert 0.0 < coefficient < 1.0
    assert agctime_seconds(coefficient) == pytest.approx(seconds, abs=1e-12)


def test_predicted_quarter_second_coefficient():
    assert agctime_coefficient(0.25) == pytest.approx(0.938005, abs=1e-06)


@pytest.mark.parametrize("coefficient", [0.0, 1.0, -0.5, 1.5])
def test_agctime_rejects_a_coefficient_no_ramp_time_maps_to(coefficient):
    with pytest.raises(ValueError):
        agctime_seconds(coefficient)


def test_agctime_rejects_a_non_positive_ramp_time():
    with pytest.raises(ValueError):
        agctime_coefficient(0.0)


def test_only_agctime_is_converted_out_of_the_device_unit():
    assert parameter_from_readback(
        "AGCTIME", MEASURED_AGCTIME_READBACKS[0.5]
    ) == pytest.approx(0.5, abs=AGCTIME_READBACK_EPSILON_SECONDS)
    assert parameter_from_readback("AGCMAXGAIN", 31.6) == 31.6
    assert parameter_from_readback("HPFONOFF", 2) == 2


def test_agctime_epsilon_covers_the_residual_but_not_a_wrong_ramp_time():
    assert readback_matches(
        "AGCTIME", 1.0, agctime_seconds(MEASURED_AGCTIME_READBACKS[1.0])
    )
    assert readback_matches(
        "AGCTIME", 0.5, agctime_seconds(MEASURED_AGCTIME_READBACKS[0.5])
    )

    # The 1.0 s coefficient answering a 0.5 s write stays a rejection.
    assert not readback_matches(
        "AGCTIME", 0.5, agctime_seconds(MEASURED_AGCTIME_READBACKS[1.0])
    )
    # A ramp time 5 ms off is more than the conversion residual explains.
    assert not readback_matches("AGCTIME", 0.5, 0.505)


def test_tolerance_table_is_relative_and_absolute():
    assert readback_tolerance("AGCTIME") == (
        DEFAULT_READBACK_TOLERANCE[0],
        AGCTIME_READBACK_EPSILON_SECONDS,
    )
    assert AGCTIME_READBACK_EPSILON_SECONDS == 2e-3
    assert readback_tolerance("AGCMAXGAIN") == DEFAULT_READBACK_TOLERANCE
    assert readback_tolerance("unlisted") == DEFAULT_READBACK_TOLERANCE

    # Relative branch: 6.3e-6 of 31.6 is far more than the absolute epsilon.
    assert readback_matches("AGCMAXGAIN", 31.6, 31.6002)
    assert not readback_matches("AGCMAXGAIN", 31.6, 31.61)

    # Absolute branch: at the bottom of the range the relative tolerance is
    # 1e-13, so only the absolute epsilon can carry a read-back.
    assert readback_matches("AGCDESIREDLEVEL", 1e-08, 1.5e-08)
    assert not readback_matches("AGCDESIREDLEVEL", 1e-08, 3e-08)


def test_integer_readback_stays_exact():
    assert readback_matches("HPFONOFF", 2, 2)
    assert not readback_matches("HPFONOFF", 2, 1)


def test_mismatch_reason_names_parameter_and_both_values():
    held = agctime_seconds(MEASURED_AGCTIME_READBACKS[1.0])
    reason = readback_mismatch_reason(
        "AGCTIME", 0.5, held, MEASURED_AGCTIME_READBACKS[1.0]
    )
    assert "AGCTIME" in reason
    assert "0.5" in reason
    assert str(held) in reason
    assert "0.9841422392055392" in reason
    assert "0.002" in reason

    assert readback_mismatch_reason("HPFONOFF", 2, 1) == (
        "HPFONOFF read-back 1 != requested 2"
    )


@pytest.mark.parametrize("preset", [name for name in PRESETS if name != "Custom"])
def test_preset_applies_against_a_device_answering_in_coefficients(preset):
    device = CoefficientDevice()

    readbacks, failure = apply_tuning_values(device, PRESETS[preset])

    assert failure is None
    assert set(readbacks) == set(TUNABLE_PARAMETERS)
    requested = PRESETS[preset]["AGCTIME"]
    assert device.written["AGCTIME"] == requested
    assert readbacks["AGCTIME"] == pytest.approx(
        requested, abs=AGCTIME_READBACK_EPSILON_SECONDS
    )
    assert readbacks["HPFONOFF"] == PRESETS[preset]["HPFONOFF"]


def test_apply_rejects_a_ramp_time_the_device_does_not_store():
    device = CoefficientDevice(held_agctime_seconds=1.0)

    readbacks, failure = apply_tuning_values(device, {"AGCTIME": 0.5})

    held = readbacks["AGCTIME"]
    assert held == pytest.approx(1.0, abs=AGCTIME_READBACK_EPSILON_SECONDS)
    assert failure == readback_mismatch_reason(
        "AGCTIME", 0.5, held, agctime_coefficient(1.0) + MEASURED_COEFFICIENT_RESIDUAL
    )
    assert "AGCTIME" in failure
    assert "0.5" in failure


def test_apply_rejects_a_readback_that_is_not_a_ramp_time_at_all():
    device = SaturatedDevice()

    readbacks, failure = apply_tuning_values(device, {"AGCTIME": 0.5})

    assert readbacks == {}
    assert "AGCTIME" in failure
    assert "0.5" in failure
    assert "1.0" in failure


def test_apply_reports_a_failing_device_and_keeps_earlier_readbacks():
    device = BrokenDevice()

    readbacks, failure = apply_tuning_values(
        device, {"HPFONOFF": 2, "AGCONOFF": 0, "ECHOONOFF": 1}
    )

    assert readbacks == {"HPFONOFF": 2}
    assert failure == "Device write/read-back failed: usb timeout"
