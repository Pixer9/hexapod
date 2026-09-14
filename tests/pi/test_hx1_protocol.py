"""Tests for Pi-side HX1 asymmetric framing primitives."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC = REPO_ROOT / "src"
sys.path.insert(0, str(SRC))

from hexapod.hx1 import (  # noqa: E402
    UINT32_MAX,
    HX1CRCError,
    HX1LineFramer,
    HX1Sequence,
    crc16_ccitt_false,
    encode_frame,
    parse_frame,
    sequence_is_newer,
)

SESSION = "A1B2C3D4"
JOINTS = tuple(range(-900, 900, 100))


class HX1ProtocolTests(unittest.TestCase):
    def test_crc_standard_check_vector(self):
        self.assertEqual(crc16_ccitt_false(b"123456789"), 0x29B1)

    def test_binary_stop_has_frozen_golden_vector(self):
        self.assertEqual(
            encode_frame(42, "STOP", SESSION).hex().upper(),
            "4858010700002A000000D4C3B2A1AD2E",
        )

    def test_binary_stop_round_trip(self):
        encoded = encode_frame(42, "STOP", SESSION)
        self.assertEqual(len(encoded), 16)
        self.assertEqual(encoded[:2], b"HX")
        self.assertNotIn(b"\n", encoded)

        parsed = parse_frame(encoded)
        self.assertEqual(parsed.seq, 42)
        self.assertEqual(parsed.message_type, "STOP")
        self.assertEqual(parsed.fields, (SESSION,))

    def test_binary_stage_is_fixed_52_bytes(self):
        encoded = encode_frame(9, "STAGE", SESSION, *JOINTS)
        self.assertEqual(len(encoded), 52)
        parsed = parse_frame(encoded)
        self.assertEqual(parsed.message_type, "STAGE")
        self.assertEqual(parsed.fields, (SESSION,) + tuple(str(v) for v in JOINTS))

    def test_binary_target_is_fixed_56_bytes(self):
        encoded = encode_frame(10, "TARGET", SESSION, 20, *JOINTS)
        self.assertEqual(len(encoded), 56)
        parsed = parse_frame(encoded)
        self.assertEqual(parsed.message_type, "TARGET")
        self.assertEqual(parsed.fields[0], SESSION)
        self.assertEqual(parsed.fields[1], "20")
        self.assertEqual(parsed.fields[2:], tuple(str(v) for v in JOINTS))

    def test_binary_crc_corruption_is_rejected(self):
        encoded = bytearray(encode_frame(1, "STAGE", SESSION, *JOINTS))
        encoded[-1] ^= 0x01
        with self.assertRaises(HX1CRCError):
            parse_frame(encoded)

    def test_ascii_telemetry_round_trip(self):
        encoded = encode_frame(7, "ACK", 6, "ARM")
        self.assertTrue(encoded.endswith(b"\n"))
        parsed = parse_frame(encoded)
        self.assertEqual(parsed.seq, 7)
        self.assertEqual(parsed.message_type, "ACK")
        self.assertEqual(parsed.fields, ("6", "ARM"))

    def test_ascii_crlf_is_tolerated(self):
        encoded = encode_frame(7, "ACK", 6, "ARM")
        parsed = parse_frame(encoded[:-1] + b"\r\n")
        self.assertEqual(parsed.message_type, "ACK")

    def test_lowercase_message_type_is_rejected_on_encode(self):
        with self.assertRaises(ValueError):
            encode_frame(1, "target", "x")

    def test_reserved_field_character_is_rejected(self):
        with self.assertRaises(ValueError):
            encode_frame(1, "EVENT", "bad|field")

    def test_sequence_wraps_modulo_uint32(self):
        sequence = HX1Sequence(UINT32_MAX)
        self.assertEqual(sequence.take(), UINT32_MAX)
        self.assertEqual(sequence.take(), 0)
        self.assertEqual(sequence.next_value, 1)

    def test_half_range_sequence_rule(self):
        self.assertTrue(sequence_is_newer(0, UINT32_MAX))
        self.assertFalse(sequence_is_newer(5, 5))
        self.assertFalse(sequence_is_newer(0x80000000, 0))

    def test_line_framer_handles_fragmented_ascii_telemetry(self):
        framer = HX1LineFramer()
        frame = encode_frame(5, "ACK", 4, "GET_STATUS")
        self.assertEqual(framer.feed(frame[:4]), ())
        self.assertEqual(framer.feed(frame[4:]), (frame,))

    def test_line_framer_discards_oversized_ascii_line(self):
        framer = HX1LineFramer()
        self.assertEqual(framer.feed(b"A" * 513), ())
        self.assertEqual(framer.framing_errors, 1)
        framer.feed(b"\n")
        valid = encode_frame(0, "EVENT", "FAULT", "PROFILE")
        self.assertEqual(framer.feed(valid), (valid,))


if __name__ == "__main__":
    unittest.main()
