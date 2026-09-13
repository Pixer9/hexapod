"""Tests for pure Pi-side HX1 framing primitives."""

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


class HX1ProtocolTests(unittest.TestCase):
    def test_crc_standard_check_vector(self):
        self.assertEqual(
            crc16_ccitt_false(b"123456789"),
            0x29B1,
        )

    def test_encode_parse_round_trip(self):
        encoded = encode_frame(
            42,
            "STOP",
            "A1B2C3D4",
        )
        parsed = parse_frame(encoded)

        self.assertEqual(parsed.seq, 42)
        self.assertEqual(parsed.message_type, "STOP")
        self.assertEqual(parsed.fields, ("A1B2C3D4",))

    def test_crlf_is_tolerated(self):
        encoded = encode_frame(3, "ARM", "A1B2C3D4")
        raw = encoded[:-1] + b"\r\n"

        parsed = parse_frame(raw)

        self.assertEqual(parsed.message_type, "ARM")

    def test_crc_corruption_is_rejected(self):
        encoded = bytearray(encode_frame(1, "ARM", "A1B2C3D4"))
        encoded[-3] = ord("0")

        with self.assertRaises(HX1CRCError):
            parse_frame(encoded)

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
        self.assertFalse(
            sequence_is_newer(
                0x80000000,
                0,
            )
        )

    def test_line_framer_handles_fragmented_input(self):
        framer = HX1LineFramer()
        frame = encode_frame(5, "GET_STATUS", "A1B2C3D4")

        self.assertEqual(framer.feed(frame[:4]), ())
        self.assertEqual(
            framer.feed(frame[4:]),
            (frame,),
        )

    def test_line_framer_discards_oversized_line(self):
        framer = HX1LineFramer()

        self.assertEqual(
            framer.feed(b"A" * 513),
            (),
        )
        self.assertEqual(framer.framing_errors, 1)

        framer.feed(b"\n")
        valid = encode_frame(0, "HELLO", 1, "profile")
        self.assertEqual(framer.feed(valid), (valid,))


if __name__ == "__main__":
    unittest.main()
