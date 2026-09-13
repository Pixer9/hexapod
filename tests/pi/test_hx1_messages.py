"""Tests for typed HX1 command/telemetry messages."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC = REPO_ROOT / "src"
sys.path.insert(0, str(SRC))

from hexapod.hx1 import (  # noqa: E402
    HX1Ack,
    HX1Info,
    HX1Status,
    HX1UnknownMessage,
    degrees_to_centidegrees,
    encode_frame,
    joint_degrees_to_centidegrees,
    parse_frame,
    parse_inbound,
)


class HX1MessageTests(unittest.TestCase):
    def test_degree_quantization_is_nearest_centidegree(self):
        self.assertEqual(degrees_to_centidegrees(12.344), 1234)
        self.assertEqual(degrees_to_centidegrees(12.345), 1235)
        self.assertEqual(degrees_to_centidegrees(-8.504), -850)
        self.assertEqual(degrees_to_centidegrees(-8.505), -851)

    def test_joint_vector_quantization_requires_18_values(self):
        with self.assertRaises(ValueError):
            joint_degrees_to_centidegrees((0.0,) * 17)

    def test_info_parses_current_schema(self):
        raw = encode_frame(
            90,
            "INFO",
            7,
            "A1B2C3D4",
            1,
            "fw-1.0",
            "mcu-123",
            "hexapod-standard-v1",
            1,
            "ABCDEF",
            18,
            "00000021",
            "DISARMED",
        )

        message = parse_inbound(parse_frame(raw))

        self.assertIsInstance(message, HX1Info)
        self.assertEqual(message.ref_seq, 7)
        self.assertEqual(message.session, "A1B2C3D4")
        self.assertEqual(message.capabilities, 0x21)
        self.assertEqual(message.state, "DISARMED")

    def test_info_allows_zero_profile_revision_for_diagnostic_fallback(self):
        raw = encode_frame(
            91,
            "INFO",
            8,
            "A1B2C3D4",
            1,
            "fw-1.0",
            "mcu-123",
            "invalid-profile",
            0,
            "0" * 64,
            18,
            "00000000",
            "FAULT",
        )

        message = parse_inbound(parse_frame(raw))

        self.assertIsInstance(message, HX1Info)
        self.assertEqual(message.profile_id, "invalid-profile")
        self.assertEqual(message.profile_revision, 0)
        self.assertEqual(message.state, "FAULT")

    def test_ack_parses(self):
        raw = encode_frame(
            10,
            "ACK",
            5,
            "STAGE",
        )

        message = parse_inbound(parse_frame(raw))

        self.assertIsInstance(message, HX1Ack)
        self.assertEqual(message.ref_seq, 5)
        self.assertEqual(message.command, "STAGE")

    def test_status_with_valid_command_exposes_vector(self):
        joints = tuple(range(18))
        raw = encode_frame(
            11,
            "STATUS",
            "A1B2C3D4",
            "ACTIVE",
            "NONE",
            123,
            10,
            20,
            5,
            7400,
            -1,
            999,
            1,
            *joints,
        )

        message = parse_inbound(parse_frame(raw))

        self.assertIsInstance(message, HX1Status)
        self.assertTrue(message.command_valid)
        self.assertEqual(message.commanded_joint_cd, joints)
        self.assertEqual(message.bus_mv, 7400)
        self.assertIsNone(message.bus_ma)

    def test_status_invalid_command_does_not_expose_placeholders(self):
        raw = encode_frame(
            11,
            "STATUS",
            "00000000",
            "DISARMED",
            "NONE",
            -1,
            -1,
            -1,
            0,
            -1,
            -1,
            999,
            0,
            *((0,) * 18),
        )

        message = parse_inbound(parse_frame(raw))

        self.assertIsInstance(message, HX1Status)
        self.assertFalse(message.command_valid)
        self.assertIsNone(message.commanded_joint_cd)
        self.assertIsNone(message.last_target_seq)
        self.assertIsNone(message.target_age_ms)
        self.assertIsNone(message.heartbeat_age_ms)

    def test_unknown_future_message_is_preserved(self):
        raw = encode_frame(
            4,
            "FUTURE",
            "one",
            "two",
        )

        message = parse_inbound(parse_frame(raw))

        self.assertIsInstance(message, HX1UnknownMessage)
        self.assertEqual(message.message_type, "FUTURE")
        self.assertEqual(message.fields, ("one", "two"))


if __name__ == "__main__":
    unittest.main()
