"""Host-side tests for outbound Servo 2040 HX1 messages."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
FIRMWARE_SRC = REPO_ROOT / "firmware" / "servo2040" / "src"
PROFILE_PATH = FIRMWARE_SRC / "config" / "actuator-profile.json"
sys.path.insert(0, str(FIRMWARE_SRC))

from hexapod_mcu.constants import JOINT_COUNT, PROTOCOL_MINOR  # noqa: E402
from hexapod_mcu.profile import load_profile  # noqa: E402
from hexapod_mcu.protocol import parse_frame  # noqa: E402
from hexapod_mcu.state_machine import RuntimeStateMachine, State  # noqa: E402
from hexapod_mcu.telemetry import TelemetryEncoder, TelemetryError  # noqa: E402


SESSION = "A1B2C3D4"
TARGET = tuple(1000 if index % 3 == 2 else 0 for index in range(JOINT_COUNT))


def ready_disarmed():
    runtime = RuntimeStateMachine()
    assert runtime.begin_self_test()
    assert runtime.complete_self_test(True)
    assert runtime.establish_session(SESSION, True)
    return runtime


class TelemetryTests(unittest.TestCase):
    def setUp(self):
        profile = load_profile(PROFILE_PATH)
        self.tx = TelemetryEncoder(
            profile=profile,
            firmware_version="0.1.0-rc1",
            mcu_id="e661410403724132",
            capabilities=0,
        )

    def test_protocol_minor_is_one(self):
        self.assertEqual(PROTOCOL_MINOR, 1)

    def test_info_schema(self):
        runtime = ready_disarmed()
        frame = self.tx.info(42, SESSION, runtime.state)

        seq, message_type, fields = parse_frame(frame)

        self.assertEqual(seq, 0)
        self.assertEqual(message_type, "INFO")
        self.assertEqual(fields[0], "42")
        self.assertEqual(fields[1], SESSION)
        self.assertEqual(fields[2], "1")
        self.assertEqual(fields[3], "0.1.0-rc1")
        self.assertEqual(fields[4], "e661410403724132")
        self.assertEqual(fields[5], "hexapod-standard-v1")
        self.assertEqual(fields[6], "1")
        self.assertEqual(len(fields[7]), 64)
        self.assertEqual(fields[8], "18")
        self.assertEqual(fields[9], "00000000")
        self.assertEqual(fields[10], State.DISARMED)

    def test_ack_schema(self):
        _, message_type, fields = parse_frame(self.tx.ack(10, "ARM"))
        self.assertEqual(message_type, "ACK")
        self.assertEqual(fields, ("10", "ARM"))

    def test_nack_schema(self):
        _, message_type, fields = parse_frame(
            self.tx.nack(10, "ARM", "ERR_NOT_READY")
        )
        self.assertEqual(message_type, "NACK")
        self.assertEqual(fields, ("10", "ARM", "ERR_NOT_READY"))

    def test_event_schema(self):
        _, message_type, fields = parse_frame(
            self.tx.event("FAULT", "LINK_TIMEOUT")
        )
        self.assertEqual(message_type, "EVENT")
        self.assertEqual(fields, ("FAULT", "LINK_TIMEOUT"))

    def test_outbound_sequence_advances(self):
        first, _, _ = parse_frame(self.tx.ack(1, "ARM"))
        second, _, _ = parse_frame(self.tx.ack(2, "START"))
        self.assertEqual(first, 0)
        self.assertEqual(second, 1)

    def test_outbound_sequence_wraps(self):
        profile = load_profile(PROFILE_PATH)
        tx = TelemetryEncoder(
            profile,
            "0.1.0-rc1",
            "e661410403724132",
            initial_seq=0xFFFFFFFF,
        )
        first, _, _ = parse_frame(tx.ack(1, "ARM"))
        second, _, _ = parse_frame(tx.ack(2, "START"))
        self.assertEqual(first, 0xFFFFFFFF)
        self.assertEqual(second, 0)

    def test_status_before_command_marks_vector_invalid(self):
        runtime = ready_disarmed()
        _, message_type, fields = parse_frame(
            self.tx.status(runtime, uptime_ms=100)
        )

        self.assertEqual(message_type, "STATUS")
        self.assertEqual(fields[0], SESSION)
        self.assertEqual(fields[1], State.DISARMED)
        self.assertEqual(fields[2], "NONE")
        self.assertEqual(fields[3], "-1")
        self.assertEqual(fields[4], "-1")
        self.assertEqual(fields[5], "-1")
        self.assertEqual(fields[9], "100")
        self.assertEqual(fields[10], "0")
        self.assertEqual(tuple(int(v) for v in fields[11:]), (0,) * 18)

    def test_status_after_arm_marks_vector_valid(self):
        runtime = ready_disarmed()
        self.assertTrue(runtime.stage_target(SESSION, TARGET, 100))
        self.assertTrue(runtime.arm(SESSION, 100))

        _, _, fields = parse_frame(
            self.tx.status(runtime, uptime_ms=125)
        )

        self.assertEqual(fields[10], "1")
        self.assertEqual(
            tuple(int(v) for v in fields[11:]),
            TARGET,
        )

    def test_status_reports_commanded_not_measured_state(self):
        runtime = ready_disarmed()
        self.assertTrue(runtime.stage_target(SESSION, TARGET, 100))
        self.assertTrue(runtime.arm(SESSION, 100))

        _, _, fields = parse_frame(
            self.tx.status(runtime, uptime_ms=125)
        )

        self.assertEqual(tuple(int(v) for v in fields[11:]), TARGET)

    def test_status_reports_target_and_heartbeat_ages(self):
        runtime = ready_disarmed()
        self.assertTrue(runtime.stage_target(SESSION, TARGET, 100))
        self.assertTrue(runtime.arm(SESSION, 100))
        self.assertTrue(runtime.heartbeat(SESSION, 110))
        self.assertTrue(runtime.start(SESSION, 110))
        self.assertTrue(runtime.accept_target(SESSION, 7, TARGET, 120))

        _, _, fields = parse_frame(
            self.tx.status(runtime, uptime_ms=150)
        )

        self.assertEqual(fields[3], "7")
        self.assertEqual(fields[4], "30")
        self.assertEqual(fields[5], "40")

    def test_status_uses_zero_session_when_none_exists(self):
        runtime = RuntimeStateMachine()

        _, _, fields = parse_frame(
            self.tx.status(runtime, uptime_ms=0)
        )

        self.assertEqual(fields[0], "00000000")

    def test_info_rejects_zero_session(self):
        with self.assertRaises(TelemetryError):
            self.tx.info(1, "00000000", State.DISARMED)

    def test_bad_event_detail_is_rejected(self):
        with self.assertRaises(TelemetryError):
            self.tx.event("FAULT", "bad detail")

    def test_commanded_vector_wrong_size_is_rejected(self):
        runtime = ready_disarmed()
        runtime.commanded_target = (0,) * 17

        with self.assertRaises(TelemetryError):
            self.tx.status(runtime, uptime_ms=100)


if __name__ == "__main__":
    unittest.main()
