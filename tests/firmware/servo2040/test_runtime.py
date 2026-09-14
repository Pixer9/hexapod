"""Integration tests for the Servo 2040 HX1 runtime coordinator."""

from __future__ import annotations

import json
import struct
import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
FIRMWARE_SRC = REPO_ROOT / "firmware" / "servo2040" / "src"
PROFILE_PATH = FIRMWARE_SRC / "config" / "actuator-profile.json"
sys.path.insert(0, str(FIRMWARE_SRC))

from hexapod_mcu.constants import (  # noqa: E402
    FAULT_HOLD_MS,
    LINK_TIMEOUT_MS,
    MOTION_TIMEOUT_MS,
)
from hexapod_mcu.hardware import (  # noqa: E402
    HardwareCompatibilityError,
    HardwareError,
)
from hexapod_mcu.profile import (  # noqa: E402
    load_profile,
    parse_profile_bytes,
)
from hexapod_mcu.protocol import (  # noqa: E402
    COMMAND_HEADER_FORMAT,
    COMMAND_MAGIC,
    COMMAND_WIRE_VERSION,
    crc16_ccitt_false,
    encode_frame,
    parse_frame,
)
from hexapod_mcu.runtime import RuntimeCoordinator  # noqa: E402
from hexapod_mcu.state_machine import (  # noqa: E402
    Fault,
    RuntimeStateMachine,
    State,
)
from hexapod_mcu.telemetry import TelemetryEncoder  # noqa: E402

SESSION_INT = 0xA1B2C3D4
SESSION = "A1B2C3D4"


def safe_target():
    return (
        0,
        0,
        1000,
        0,
        0,
        1000,
        0,
        0,
        1000,
        0,
        0,
        1000,
        0,
        0,
        1000,
        0,
        0,
        1000,
    )


def qualified_profile(rate_cd_s=25000):
    data = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))
    data["profile_revision"] += 1

    for joint in data["joints"]:
        joint["max_rate_cd_s"] = rate_cd_s
        joint["servo_min_cd"] = max(joint["servo_min_cd"], -9000)
        joint["servo_max_cd"] = min(joint["servo_max_cd"], 9000)

    return parse_profile_bytes(
        json.dumps(
            data,
            separators=(",", ":"),
        ).encode("utf-8")
    )


class FakeHardware:
    def __init__(self):
        self.enabled = False
        self.compatible = True
        self.calls = []
        self.last_target = None
        self.fail_enable = False
        self.fail_apply = False
        self.fail_disable = False

    def require_profile_compatible(self, profile):
        self.calls.append(("compatibility",))
        if not self.compatible:
            raise HardwareCompatibilityError("injected mismatch")
        return True

    def enable_at_target(self, target):
        self.calls.append(("enable", tuple(target)))
        if self.fail_enable:
            raise HardwareError("injected enable failure")
        self.enabled = True
        self.last_target = tuple(target)

    def apply_target(self, target):
        self.calls.append(("apply", tuple(target)))
        if self.fail_apply:
            raise HardwareError("injected apply failure")
        if not self.enabled:
            raise HardwareError("not enabled")
        self.last_target = tuple(target)

    def force_disabled(self):
        self.calls.append(("disable",))
        if self.fail_disable:
            raise HardwareError("injected disable failure")
        self.enabled = False
        self.last_target = None


def make_runtime(profile=None, do_self_test=True):
    if profile is None:
        profile = qualified_profile()

    sm = RuntimeStateMachine()
    hw = FakeHardware()
    tx = TelemetryEncoder(
        profile,
        "0.1.0-rc1",
        "e661410403724132",
    )
    runtime = RuntimeCoordinator(
        profile=profile,
        state_machine=sm,
        hardware=hw,
        telemetry=tx,
        session_factory=lambda: SESSION_INT,
    )

    if do_self_test:
        assert runtime.perform_self_test(0)

    return runtime, sm, hw


def send(runtime, seq, message_type, *fields, now_ms=0):
    frame = encode_frame(seq, message_type, *fields)
    return runtime.handle_frame(frame, now_ms)


def response_types(responses):
    return tuple(parse_frame(frame)[1] for frame in responses)


def response_fields(response):
    return parse_frame(response)[2]


def hello(runtime, seq=1, now_ms=10, profile_id=None):
    if profile_id is None:
        profile_id = runtime.profile.profile_id
    return send(
        runtime,
        seq,
        "HELLO",
        1,
        profile_id,
        now_ms=now_ms,
    )


def stage(runtime, seq=2, now_ms=20, target=None):
    if target is None:
        target = safe_target()
    return send(
        runtime,
        seq,
        "STAGE",
        SESSION,
        *target,
        now_ms=now_ms,
    )


def heartbeat(runtime, seq=3, now_ms=30):
    return send(
        runtime,
        seq,
        "HEARTBEAT",
        SESSION,
        now_ms,
        now_ms=now_ms,
    )


def arm(runtime, seq=4, now_ms=40):
    return send(
        runtime,
        seq,
        "ARM",
        SESSION,
        now_ms=now_ms,
    )


def start(runtime, seq=5, now_ms=50):
    return send(
        runtime,
        seq,
        "START",
        SESSION,
        now_ms=now_ms,
    )


def ready_active(rate_cd_s=25000):
    runtime, sm, hw = make_runtime(qualified_profile(rate_cd_s))
    hello(runtime)
    stage(runtime)
    heartbeat(runtime)
    arm(runtime)
    start(runtime)
    assert sm.state == State.ACTIVE
    assert hw.enabled
    return runtime, sm, hw


class SelfTestTests(unittest.TestCase):
    def test_unqualified_profile_fails_safe(self):
        data = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))
        data["joints"][0]["max_rate_cd_s"] = None

        profile = parse_profile_bytes(
            json.dumps(
                data,
                separators=(",", ":"),
            ).encode("utf-8")
        )

        runtime, sm, hw = make_runtime(
            profile,
            do_self_test=False,
        )

        self.assertFalse(runtime.perform_self_test(0))
        self.assertEqual(sm.state, State.FAULT)
        self.assertEqual(sm.fault, Fault.PROFILE)
        self.assertFalse(hw.enabled)

    def test_current_qualified_profile_reaches_disarmed(self):
        runtime, sm, hw = make_runtime(
            load_profile(PROFILE_PATH),
            do_self_test=False,
        )

        self.assertTrue(runtime.perform_self_test(0))
        self.assertEqual(sm.state, State.DISARMED)
        self.assertEqual(sm.fault, Fault.NONE)
        self.assertFalse(hw.enabled)


class HelloTests(unittest.TestCase):
    def test_hello_establishes_session_and_returns_info(self):
        runtime, sm, _ = make_runtime()

        responses = hello(runtime)

        self.assertEqual(response_types(responses), ("INFO",))
        self.assertEqual(sm.session_id, SESSION)
        self.assertTrue(sm.session_profile_match)

    def test_profile_mismatch_is_reported_but_session_exists(self):
        runtime, sm, _ = make_runtime()

        responses = hello(runtime, profile_id="wrong-profile")

        self.assertEqual(response_types(responses), ("INFO",))
        self.assertEqual(sm.session_id, SESSION)
        self.assertFalse(sm.session_profile_match)

    def test_new_hello_while_armed_disarms_hardware(self):
        runtime, sm, hw = make_runtime()
        hello(runtime)
        stage(runtime)
        heartbeat(runtime)
        arm(runtime)

        self.assertEqual(sm.state, State.ARMED)
        self.assertTrue(hw.enabled)

        hello(runtime, seq=9, now_ms=100)

        self.assertEqual(sm.state, State.DISARMED)
        self.assertFalse(hw.enabled)


class FramingTests(unittest.TestCase):
    def test_bad_crc_is_dropped_without_nack(self):
        runtime, _, _ = make_runtime()

        responses = runtime.handle_frame(
            b"HX1|1|HELLO|1|hexapod-standard-v1|0000\n",
            10,
        )

        self.assertEqual(responses, ())
        self.assertEqual(runtime.protocol_errors, 1)

    def test_unknown_binary_command_gets_unsupported_nack(self):
        runtime, _, _ = make_runtime()

        body = struct.pack(
            COMMAND_HEADER_FORMAT,
            COMMAND_MAGIC,
            COMMAND_WIRE_VERSION,
            0x7F,  # deliberately unknown command ID
            0,  # payload length
            1,  # sequence
            SESSION_INT,
        )
        frame = body + struct.pack(
            "<H",
            crc16_ccitt_false(body),
        )

        responses = runtime.handle_frame(
            frame,
            10,
        )

        self.assertEqual(
            response_types(responses),
            ("NACK",),
        )
        self.assertEqual(
            response_fields(responses[0])[-1],
            "ERR_UNSUPPORTED",
        )


class StageArmTests(unittest.TestCase):
    def test_stage_acknowledges_valid_target(self):
        runtime, sm, _ = make_runtime()
        hello(runtime)

        responses = stage(runtime)

        self.assertEqual(response_types(responses), ("ACK",))
        self.assertEqual(sm.staged_target, safe_target())

    def test_stage_limit_violation_is_atomic(self):
        runtime, sm, _ = make_runtime()
        hello(runtime)
        good = safe_target()
        stage(runtime, target=good)
        previous = sm.staged_target

        bad = list(good)
        bad[0] = runtime.profile.joint(0).logical_max_cd + 1

        responses = stage(
            runtime,
            seq=9,
            now_ms=25,
            target=bad,
        )

        self.assertEqual(response_types(responses), ("NACK",))
        self.assertEqual(
            response_fields(responses[0])[-1],
            "ERR_LIMIT",
        )
        self.assertEqual(sm.staged_target, previous)

    def test_arm_enables_exact_mapped_target(self):
        runtime, sm, hw = make_runtime()
        hello(runtime)
        stage(runtime)
        heartbeat(runtime)

        responses = arm(runtime)

        self.assertEqual(response_types(responses), ("ACK",))
        self.assertEqual(sm.state, State.ARMED)
        self.assertTrue(hw.enabled)
        self.assertEqual(hw.calls[-1][0], "enable")

    def test_arm_rejects_profile_mismatch(self):
        runtime, sm, hw = make_runtime()
        hello(runtime, profile_id="wrong-profile")
        stage(runtime)
        heartbeat(runtime)

        responses = arm(runtime)

        self.assertEqual(response_types(responses), ("NACK",))
        self.assertEqual(
            response_fields(responses[0])[-1],
            "ERR_PROFILE_MISMATCH",
        )
        self.assertEqual(sm.state, State.DISARMED)
        self.assertFalse(hw.enabled)

    def test_hardware_enable_failure_latches_fault(self):
        runtime, sm, hw = make_runtime()
        hello(runtime)
        stage(runtime)
        heartbeat(runtime)
        hw.fail_enable = True

        responses = arm(runtime)

        self.assertEqual(
            response_types(responses),
            ("NACK", "EVENT"),
        )
        self.assertEqual(sm.state, State.FAULT)
        self.assertEqual(sm.fault, Fault.HARDWARE)
        self.assertFalse(hw.enabled)


class ActiveTargetTests(unittest.TestCase):
    def test_start_requires_fresh_heartbeat(self):
        runtime, sm, _ = make_runtime()
        hello(runtime)
        stage(runtime)
        arm(runtime)

        responses = start(runtime)

        self.assertEqual(response_types(responses), ("NACK",))
        self.assertEqual(sm.state, State.ARMED)

    def test_valid_target_is_applied_without_ack(self):
        runtime, sm, hw = ready_active()
        target = list(safe_target())
        target[0] = 100  # 100 cd over 20 ms => 5000 cd/s

        responses = send(
            runtime,
            6,
            "TARGET",
            SESSION,
            20,
            *target,
            now_ms=70,
        )

        self.assertEqual(responses, ())
        self.assertEqual(sm.last_target_seq, 6)
        self.assertEqual(sm.last_target_at_ms, 70)
        self.assertEqual(hw.calls[-1][0], "apply")

    def test_rate_violation_is_rejected_without_applying(self):
        runtime, sm, hw = ready_active(rate_cd_s=25000)
        calls_before = len(hw.calls)

        target = list(safe_target())
        target[0] = 600  # 600 cd / 20 ms = 30,000 cd/s

        responses = send(
            runtime,
            6,
            "TARGET",
            SESSION,
            20,
            *target,
            now_ms=70,
        )

        self.assertEqual(response_types(responses), ("NACK",))
        self.assertEqual(
            response_fields(responses[0])[-1],
            "ERR_RATE_LIMIT",
        )
        self.assertEqual(len(hw.calls), calls_before)
        self.assertIsNone(sm.last_target_at_ms)

    def test_duplicate_target_sequence_does_not_refresh_watchdog(self):
        runtime, sm, hw = ready_active()

        send(
            runtime,
            6,
            "TARGET",
            SESSION,
            20,
            *safe_target(),
            now_ms=70,
        )
        calls_before = len(hw.calls)

        responses = send(
            runtime,
            6,
            "TARGET",
            SESSION,
            20,
            *safe_target(),
            now_ms=90,
        )

        self.assertEqual(response_types(responses), ("NACK",))
        self.assertEqual(
            response_fields(responses[0])[-1],
            "ERR_SEQ",
        )
        self.assertEqual(sm.last_target_at_ms, 70)
        self.assertEqual(len(hw.calls), calls_before)

    def test_hardware_apply_failure_latches_fault(self):
        runtime, sm, hw = ready_active()
        hw.fail_apply = True

        responses = send(
            runtime,
            6,
            "TARGET",
            SESSION,
            20,
            *safe_target(),
            now_ms=70,
        )

        self.assertEqual(
            response_types(responses),
            ("NACK", "EVENT"),
        )
        self.assertEqual(sm.state, State.FAULT)
        self.assertEqual(sm.fault, Fault.HARDWARE)
        self.assertFalse(hw.enabled)


class StopDisarmTests(unittest.TestCase):
    def test_stop_returns_to_armed_without_disabling(self):
        runtime, sm, hw = ready_active()

        responses = send(
            runtime,
            10,
            "STOP",
            SESSION,
            now_ms=80,
        )

        self.assertEqual(response_types(responses), ("ACK",))
        self.assertEqual(sm.state, State.ARMED)
        self.assertTrue(hw.enabled)

    def test_disarm_releases_outputs(self):
        runtime, sm, hw = ready_active()

        responses = send(
            runtime,
            10,
            "DISARM",
            SESSION,
            now_ms=80,
        )

        self.assertEqual(response_types(responses), ("ACK",))
        self.assertEqual(sm.state, State.DISARMED)
        self.assertFalse(hw.enabled)


class EstopTests(unittest.TestCase):
    def test_wrong_session_estop_is_still_honored(self):
        runtime, sm, hw = ready_active()

        responses = send(
            runtime,
            10,
            "ESTOP",
            "DEADBEEF",
            "operator",
            now_ms=80,
        )

        self.assertEqual(
            response_types(responses),
            ("ACK", "EVENT"),
        )
        self.assertEqual(sm.state, State.ESTOP)
        self.assertFalse(hw.enabled)

    def test_clear_estop_returns_only_to_disarmed(self):
        runtime, sm, hw = ready_active()
        send(
            runtime,
            10,
            "ESTOP",
            "00000000",
            "operator",
            now_ms=80,
        )

        responses = send(
            runtime,
            11,
            "CLEAR_ESTOP",
            SESSION,
            now_ms=90,
        )

        self.assertEqual(
            response_types(responses),
            ("ACK", "EVENT"),
        )
        self.assertEqual(sm.state, State.DISARMED)
        self.assertFalse(hw.enabled)


class StatusTests(unittest.TestCase):
    def test_get_status_uses_current_session(self):
        runtime, _, _ = make_runtime()
        hello(runtime)

        responses = send(
            runtime,
            10,
            "GET_STATUS",
            SESSION,
            now_ms=100,
        )

        self.assertEqual(response_types(responses), ("STATUS",))

    def test_status_before_hello_accepts_zero_session(self):
        runtime, _, _ = make_runtime()

        responses = send(
            runtime,
            10,
            "GET_STATUS",
            "00000000",
            now_ms=100,
        )

        self.assertEqual(response_types(responses), ("STATUS",))


class WatchdogIntegrationTests(unittest.TestCase):
    def test_link_watchdog_enters_fault_then_releases_after_hold(self):
        runtime, sm, hw = make_runtime()
        hello(runtime, now_ms=10)
        stage(runtime, now_ms=20)
        heartbeat(runtime, now_ms=30)
        arm(runtime, now_ms=40)

        fault_at = 30 + LINK_TIMEOUT_MS
        responses = runtime.tick(fault_at)

        self.assertEqual(response_types(responses), ("EVENT",))
        self.assertEqual(sm.state, State.FAULT)
        self.assertTrue(hw.enabled)

        runtime.tick(fault_at + FAULT_HOLD_MS - 1)
        self.assertTrue(hw.enabled)

        runtime.tick(fault_at + FAULT_HOLD_MS)
        self.assertFalse(hw.enabled)

    def test_motion_watchdog_does_not_refresh_from_heartbeat(self):
        runtime, sm, hw = ready_active()

        heartbeat(runtime, seq=20, now_ms=100)

        fault_at = 50 + MOTION_TIMEOUT_MS
        responses = runtime.tick(fault_at)

        self.assertEqual(response_types(responses), ("EVENT",))
        self.assertEqual(sm.fault, Fault.MOTION_TIMEOUT)
        self.assertTrue(hw.enabled)

    def test_watchdog_fault_can_clear_after_fresh_heartbeat(self):
        runtime, sm, hw = make_runtime()
        hello(runtime, now_ms=10)
        stage(runtime, now_ms=20)
        heartbeat(runtime, now_ms=30)
        arm(runtime, now_ms=40)

        fault_at = 30 + LINK_TIMEOUT_MS
        runtime.tick(fault_at)
        heartbeat(runtime, seq=20, now_ms=fault_at + 10)

        responses = send(
            runtime,
            21,
            "CLEAR_FAULT",
            SESSION,
            now_ms=fault_at + 20,
        )

        self.assertEqual(
            response_types(responses),
            ("ACK", "EVENT"),
        )
        self.assertEqual(sm.state, State.DISARMED)
        self.assertFalse(hw.enabled)


if __name__ == "__main__":
    unittest.main()
