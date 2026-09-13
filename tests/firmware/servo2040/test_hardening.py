"""Regression tests for the final pre-shelving MCU hardening pass."""

from __future__ import annotations

import importlib
import json
import sys
import unittest
from pathlib import Path
from types import ModuleType

REPO_ROOT = Path(__file__).resolve().parents[3]
FIRMWARE_SRC = REPO_ROOT / "firmware" / "servo2040" / "src"
PROFILE_PATH = FIRMWARE_SRC / "config" / "actuator-profile.json"
sys.path.insert(0, str(FIRMWARE_SRC))

firmware_main = importlib.import_module("main")

import hexapod_mcu.hardware as hardware_module  # noqa: E402
import hexapod_mcu.transport as transport_module  # noqa: E402
from hexapod_mcu.constants import (  # noqa: E402
    JOINT_COUNT,
    LINK_TIMEOUT_MS,
    MAX_MESSAGE_TYPE_CHARS,
    MOTION_TIMEOUT_MS,
)
from hexapod_mcu.hardware import (  # noqa: E402
    HardwareError,
    ServoOutputHardware,
)
from hexapod_mcu.profile import parse_profile_bytes  # noqa: E402
from hexapod_mcu.protocol import (  # noqa: E402
    FrameFormatError,
    crc16_ccitt_false,
    encode_frame,
    parse_frame,
)
from hexapod_mcu.runtime import RuntimeCoordinator  # noqa: E402
from hexapod_mcu.state_machine import (  # noqa: E402
    Error,
    Fault,
    RuntimeStateMachine,
    State,
)
from hexapod_mcu.telemetry import TelemetryEncoder  # noqa: E402


SESSION = "A1B2C3D4"
SESSION_INT = 0xA1B2C3D4


def safe_target():
    return (
        0, 0, 1000,
        0, 0, 1000,
        0, 0, 1000,
        0, 0, 1000,
        0, 0, 1000,
        0, 0, 1000,
    )


def qualified_profile(rate_cd_s=25000):
    data = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))
    data["profile_revision"] += 1

    for joint in data["joints"]:
        joint["max_rate_cd_s"] = rate_cd_s
        joint["servo_min_cd"] = max(joint["servo_min_cd"], -9000)
        joint["servo_max_cd"] = min(joint["servo_max_cd"], 9000)

    return parse_profile_bytes(
        json.dumps(data, separators=(",", ":")).encode("utf-8")
    )


class FakeHardware:
    def __init__(self):
        self.enabled = False
        self.calls = []
        self.fail_disable = False

    def require_profile_compatible(self, profile):
        return True

    def enable_at_target(self, target):
        self.calls.append(("enable", tuple(target)))
        self.enabled = True

    def apply_target(self, target):
        self.calls.append(("apply", tuple(target)))
        if self.enabled is not True:
            raise HardwareError("not enabled")

    def force_disabled(self):
        self.calls.append(("disable",))
        if self.fail_disable:
            self.enabled = None
            raise HardwareError("injected disable failure")
        self.enabled = False


class MinimalCluster:
    def __init__(self):
        self.fail_disable = False
        self.fail_value_channel = None

    def count(self):
        return 18

    def min_value(self, channel):
        return -90.0

    def max_value(self, channel):
        return 90.0

    def value(self, channel, value, load=True):
        if channel == self.fail_value_channel:
            raise RuntimeError("injected value failure")

    def load(self):
        pass

    def disable_all(self, load=True):
        if self.fail_disable:
            raise RuntimeError("injected disable failure")


def make_runtime():
    profile = qualified_profile()
    sm = RuntimeStateMachine()
    hw = FakeHardware()
    tx = TelemetryEncoder(profile, "0.1.0-rc1", "mcu")
    runtime = RuntimeCoordinator(
        profile=profile,
        state_machine=sm,
        hardware=hw,
        telemetry=tx,
        session_factory=lambda: SESSION_INT,
    )
    assert runtime.perform_self_test(0)
    return runtime, sm, hw


def send(runtime, seq, command, *fields, now_ms):
    return runtime.handle_frame(
        encode_frame(seq, command, *fields),
        now_ms,
    )


def establish_armed(runtime, now_hello=10, now_stage=20, now_heartbeat=30, now_arm=40):
    send(
        runtime,
        1,
        "HELLO",
        1,
        runtime.profile.profile_id,
        now_ms=now_hello,
    )
    send(
        runtime,
        2,
        "STAGE",
        SESSION,
        *safe_target(),
        now_ms=now_stage,
    )
    send(
        runtime,
        3,
        "HEARTBEAT",
        SESSION,
        now_heartbeat,
        now_ms=now_heartbeat,
    )
    send(runtime, 4, "ARM", SESSION, now_ms=now_arm)


def establish_active(runtime):
    establish_armed(runtime)
    send(runtime, 5, "START", SESSION, now_ms=50)


def response_types(responses):
    return tuple(parse_frame(frame)[1] for frame in responses)


class StateMachineHardeningTests(unittest.TestCase):
    def test_fault_from_disarmed_never_creates_pwm_hold_authority(self):
        sm = RuntimeStateMachine()
        self.assertTrue(sm.begin_self_test())
        self.assertTrue(sm.complete_self_test(True))
        self.assertTrue(sm.establish_session(SESSION, True))
        self.assertTrue(sm.stage_target(SESSION, safe_target(), 10))
        self.assertTrue(sm.arm(SESSION, 10))
        self.assertTrue(sm.disarm(SESSION))

        # commanded_target is deliberately retained after DISARM.
        self.assertIsNotNone(sm.commanded_target)
        sm.enter_fault(Fault.INTERNAL, now_ms=20, hold_pwm=True)

        self.assertEqual(sm.state, State.FAULT)
        self.assertFalse(sm.fault_hold_pwm)
        self.assertFalse(sm.pwm_should_be_enabled(20))

    def test_clear_estop_defaults_to_not_clearing_retained_fault(self):
        sm = RuntimeStateMachine()
        self.assertTrue(sm.begin_self_test())
        self.assertTrue(sm.complete_self_test(True))
        self.assertTrue(sm.establish_session(SESSION, True))
        sm.enter_fault(Fault.HARDWARE, now_ms=10, hold_pwm=False)
        sm.estop("operator", now_ms=20)

        result = sm.clear_estop(SESSION, released=True)

        self.assertFalse(result)
        self.assertEqual(result.error, Error.FAULT_ACTIVE)
        self.assertEqual(sm.state, State.ESTOP)


class HardwareTruthfulnessTests(unittest.TestCase):
    def test_failed_disable_marks_output_state_unknown(self):
        cluster = MinimalCluster()
        hardware = ServoOutputHardware(cluster)
        hardware.force_disabled()
        target = (0,) * 18
        hardware.enable_at_target(target)
        self.assertIs(hardware.enabled, True)

        cluster.fail_disable = True
        with self.assertRaises(HardwareError):
            hardware.force_disabled()

        self.assertIsNone(hardware.enabled)
        self.assertEqual(hardware.last_channel_target_cd, target)

    def test_failed_commit_and_failed_emergency_disable_marks_unknown(self):
        cluster = MinimalCluster()
        hardware = ServoOutputHardware(cluster)
        hardware.force_disabled()
        hardware.enable_at_target((0,) * 18)

        cluster.fail_value_channel = 5
        cluster.fail_disable = True

        with self.assertRaises(HardwareError):
            hardware.apply_target((100,) * 18)

        self.assertIsNone(hardware.enabled)
        self.assertEqual(hardware.last_channel_target_cd, (0,) * 18)

    def test_self_test_establishes_known_disabled_from_initial_unknown(self):
        profile = qualified_profile()
        sm = RuntimeStateMachine()
        cluster = MinimalCluster()
        hardware = ServoOutputHardware(cluster)
        tx = TelemetryEncoder(profile, "0.1.0-rc1", "mcu")
        runtime = RuntimeCoordinator(
            profile=profile,
            state_machine=sm,
            hardware=hardware,
            telemetry=tx,
            session_factory=lambda: SESSION_INT,
        )

        self.assertIsNone(hardware.enabled)
        self.assertTrue(runtime.perform_self_test(0))
        self.assertFalse(hardware.enabled)
        self.assertEqual(sm.state, State.DISARMED)

    def test_runtime_retries_disable_when_output_state_is_unknown(self):
        runtime, sm, hw = make_runtime()
        hw.enabled = None
        hw.fail_disable = True

        responses = runtime.tick(100)

        self.assertEqual(sm.state, State.FAULT)
        self.assertEqual(sm.fault, Fault.HARDWARE)
        self.assertEqual(hw.calls[-1], ("disable",))
        self.assertEqual(response_types(responses), ("EVENT",))

    def test_repeated_unknown_disable_does_not_flood_fault_events(self):
        runtime, sm, hw = make_runtime()
        hw.enabled = None
        hw.fail_disable = True

        first = runtime.tick(100)
        second = runtime.tick(101)
        third = runtime.tick(102)

        self.assertEqual(response_types(first), ("EVENT",))
        self.assertEqual(second, ())
        self.assertEqual(third, ())
        self.assertEqual(sm.fault, Fault.HARDWARE)
        self.assertEqual(
            [call for call in hw.calls if call == ("disable",)],
            [("disable",), ("disable",), ("disable",), ("disable",)],
        )


class DeadlineOrderingTests(unittest.TestCase):
    def test_heartbeat_one_ms_before_deadline_can_refresh_link(self):
        runtime, sm, _ = make_runtime()
        establish_armed(runtime)

        now = 30 + LINK_TIMEOUT_MS - 1
        responses = send(
            runtime,
            10,
            "HEARTBEAT",
            SESSION,
            now,
            now_ms=now,
        )

        self.assertEqual(responses, ())
        self.assertEqual(sm.state, State.ARMED)
        self.assertEqual(sm.last_heartbeat_at_ms, now)

    def test_heartbeat_at_deadline_cannot_rescue_expired_link(self):
        runtime, sm, _ = make_runtime()
        establish_armed(runtime)

        now = 30 + LINK_TIMEOUT_MS
        responses = send(
            runtime,
            10,
            "HEARTBEAT",
            SESSION,
            now,
            now_ms=now,
        )

        self.assertEqual(sm.state, State.FAULT)
        self.assertEqual(sm.fault, Fault.LINK_TIMEOUT)
        self.assertEqual(sm.last_heartbeat_at_ms, now)
        self.assertIn("EVENT", response_types(responses))

    def test_start_rejects_heartbeat_exactly_at_timeout_boundary(self):
        sm = RuntimeStateMachine()
        self.assertTrue(sm.begin_self_test())
        self.assertTrue(sm.complete_self_test(True))
        self.assertTrue(sm.establish_session(SESSION, True))
        self.assertTrue(sm.stage_target(SESSION, safe_target(), 0))
        self.assertTrue(sm.arm(SESSION, 0))
        self.assertTrue(sm.heartbeat(SESSION, 10))

        result = sm.start(SESSION, 10 + LINK_TIMEOUT_MS)

        self.assertFalse(result)
        self.assertEqual(result.error, Error.NOT_READY)
        self.assertEqual(sm.state, State.ARMED)

    def test_late_disarm_is_still_allowed_to_remove_authority(self):
        runtime, sm, hw = make_runtime()
        establish_armed(runtime)

        now = 30 + LINK_TIMEOUT_MS + 50
        responses = send(
            runtime,
            10,
            "DISARM",
            SESSION,
            now_ms=now,
        )

        self.assertEqual(response_types(responses), ("ACK",))
        self.assertEqual(sm.state, State.DISARMED)
        self.assertEqual(sm.fault, Fault.NONE)
        self.assertFalse(hw.enabled)

    def test_target_at_motion_deadline_cannot_rescue_expired_motion(self):
        runtime, sm, hw = make_runtime()
        establish_active(runtime)
        calls_before = tuple(hw.calls)

        now = 50 + MOTION_TIMEOUT_MS
        responses = send(
            runtime,
            10,
            "TARGET",
            SESSION,
            20,
            *safe_target(),
            now_ms=now,
        )

        self.assertEqual(sm.state, State.FAULT)
        self.assertEqual(sm.fault, Fault.MOTION_TIMEOUT)
        self.assertIsNone(sm.last_target_at_ms)
        self.assertEqual(tuple(hw.calls), calls_before)
        self.assertEqual(response_types(responses), ("EVENT", "NACK"))


class ProtocolContainmentTests(unittest.TestCase):
    def test_message_type_length_is_bounded_on_encode(self):
        with self.assertRaises(ValueError):
            encode_frame(1, "A" * (MAX_MESSAGE_TYPE_CHARS + 1))

    def test_message_type_length_is_bounded_on_parse(self):
        message_type = "A" * (MAX_MESSAGE_TYPE_CHARS + 1)
        body = ("HX1|1|%s" % message_type).encode("ascii")
        frame = body + ("|%04X\n" % crc16_ccitt_false(body)).encode("ascii")

        with self.assertRaises(FrameFormatError):
            parse_frame(frame)

    def test_bad_estop_reason_is_semantic_nack_not_internal_fault(self):
        runtime, sm, _ = make_runtime()
        send(runtime, 1, "HELLO", 1, runtime.profile.profile_id, now_ms=10)

        responses = send(
            runtime,
            2,
            "ESTOP",
            SESSION,
            "bad reason",
            now_ms=20,
        )

        self.assertEqual(response_types(responses), ("NACK",))
        _, _, fields = parse_frame(responses[0])
        self.assertEqual(fields[-1], Error.BAD_VALUE)
        self.assertEqual(sm.state, State.DISARMED)
        self.assertEqual(runtime.internal_errors, 0)

    def test_reporting_failure_is_contained_and_forces_safe_state(self):
        runtime, sm, hw = make_runtime()
        original_dispatch = runtime._dispatch
        original_fail = runtime._fail_internal

        def explode_dispatch(*args, **kwargs):
            raise RuntimeError("primary failure")

        def explode_report(*args, **kwargs):
            raise RuntimeError("reporting failure")

        runtime._dispatch = explode_dispatch
        runtime._fail_internal = explode_report
        try:
            responses = runtime.handle_frame(
                encode_frame(1, "GET_STATUS", "00000000"),
                100,
            )
        finally:
            runtime._dispatch = original_dispatch
            runtime._fail_internal = original_fail

        self.assertEqual(responses, ())
        self.assertEqual(sm.state, State.FAULT)
        self.assertEqual(sm.fault, Fault.INTERNAL)
        self.assertFalse(hw.enabled)


class MainFailSafeTests(unittest.TestCase):
    def test_ctrl_c_is_reserved_as_maintenance_escape(self):
        calls = []

        class FakeMicroPython:
            def kbd_intr(self, value):
                calls.append(value)

        firmware_main._enable_maintenance_keyboard_interrupt(
            FakeMicroPython()
        )
        self.assertEqual(calls, [3])

    def test_build_failure_after_hardware_acquisition_disables_before_reraising(self):
        hardware = FakeHardware()

        fake_machine = ModuleType("machine")
        fake_micropython = ModuleType("micropython")
        fake_micropython.kbd_intr = lambda value: None

        had_machine = "machine" in sys.modules
        previous_machine = sys.modules.get("machine")
        had_micropython = "micropython" in sys.modules
        previous_micropython = sys.modules.get("micropython")

        original_hardware_factory = hardware_module.create_servo2040_hardware
        original_transport_factory = transport_module.create_usb_cdc_transport

        def fail_transport():
            raise KeyboardInterrupt()

        hardware_module.create_servo2040_hardware = lambda: hardware
        transport_module.create_usb_cdc_transport = fail_transport
        sys.modules["machine"] = fake_machine
        sys.modules["micropython"] = fake_micropython

        try:
            with self.assertRaises(KeyboardInterrupt):
                firmware_main.build_application()
        finally:
            hardware_module.create_servo2040_hardware = original_hardware_factory
            transport_module.create_usb_cdc_transport = original_transport_factory

            if had_machine:
                sys.modules["machine"] = previous_machine
            else:
                sys.modules.pop("machine", None)

            if had_micropython:
                sys.modules["micropython"] = previous_micropython
            else:
                sys.modules.pop("micropython", None)

        self.assertEqual(hardware.calls, [("disable",)])
        self.assertFalse(hardware.enabled)

    def test_main_finally_disables_hardware_on_keyboard_interrupt(self):
        class Hardware:
            def __init__(self):
                self.disable_calls = 0

            def force_disabled(self):
                self.disable_calls += 1

        class App:
            def __init__(self):
                self.hardware = Hardware()

            def run_forever(self):
                raise KeyboardInterrupt()

        app = App()
        original_builder = firmware_main.build_application
        firmware_main.build_application = lambda: app
        try:
            with self.assertRaises(KeyboardInterrupt):
                firmware_main.main()
        finally:
            firmware_main.build_application = original_builder

        self.assertEqual(app.hardware.disable_calls, 1)

    def test_per_frame_runtime_exception_latches_fault_in_scheduler(self):
        class Clock:
            def ticks_ms(self):
                return 100

            def ticks_diff(self, now, then):
                return now - then

            def sleep_ms(self, duration):
                pass

        class StateMachine:
            def __init__(self):
                self.state = State.DISARMED
                self.fault = Fault.NONE

            def enter_fault(self, fault_code, now_ms, hold_pwm):
                self.state = State.FAULT
                self.fault = fault_code

        class Telemetry:
            def event(self, event_type, detail):
                return encode_frame(99, "EVENT", event_type, detail)

        class Runtime:
            def __init__(self):
                self.state_machine = StateMachine()
                self.telemetry = Telemetry()

            def handle_frame(self, frame, now_ms):
                raise RuntimeError("injected")

            def tick(self, now_ms):
                return ()

            def status_frame(self, now_ms):
                return encode_frame(100, "EVENT", "STATUS", now_ms)

        class Transport:
            def __init__(self):
                self.sent = []

            def poll_frames(self):
                return (encode_frame(1, "GET_STATUS", "00000000"),)

            def send_frame(self, frame):
                self.sent.append(frame)

        class Hardware:
            enabled = False

            def __init__(self):
                self.disable_calls = 0

            def force_disabled(self):
                self.disable_calls += 1

        runtime = Runtime()
        hardware = Hardware()
        app = firmware_main.FirmwareApp(
            runtime,
            Transport(),
            hardware,
            Clock(),
        )

        app.run_once()

        self.assertEqual(runtime.state_machine.state, State.FAULT)
        self.assertEqual(runtime.state_machine.fault, Fault.INTERNAL)
        self.assertGreaterEqual(hardware.disable_calls, 1)
        self.assertEqual(app.loop_errors, 1)


if __name__ == "__main__":
    unittest.main()
