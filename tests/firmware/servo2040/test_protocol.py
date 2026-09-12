"""Host-side tests for the pure Servo 2040 protocol core."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
FIRMWARE_SRC = REPO_ROOT / "firmware" / "servo2040" / "src"
sys.path.insert(0, str(FIRMWARE_SRC))

from hexapod_mcu.protocol import (  # noqa: E402
    CRCError,
    FrameFormatError,
    LineFramer,
    crc16_ccitt_false,
    encode_frame,
    parse_frame,
    sequence_is_newer,
)


class CRC16Tests(unittest.TestCase):
    def test_standard_check_vector(self):
        self.assertEqual(crc16_ccitt_false(b"123456789"), 0x29B1)

    def test_documented_stop_body(self):
        body = b"HX1|42|STOP|A1B2C3D4"
        self.assertEqual(crc16_ccitt_false(body), 0x972B)


class FrameTests(unittest.TestCase):
    def test_encode_stop(self):
        self.assertEqual(
            encode_frame(42, "STOP", "A1B2C3D4"),
            b"HX1|42|STOP|A1B2C3D4|972B\n",
        )

    def test_round_trip(self):
        raw = encode_frame(7, "HEARTBEAT", "A1B2C3D4", 12345)
        self.assertEqual(
            parse_frame(raw),
            (7, "HEARTBEAT", ("A1B2C3D4", "12345")),
        )

    def test_crlf_is_tolerated(self):
        raw = encode_frame(42, "STOP", "A1B2C3D4")
        crlf = raw[:-1] + b"\r\n"
        self.assertEqual(
            parse_frame(crlf),
            (42, "STOP", ("A1B2C3D4",)),
        )

    def test_bad_crc_is_rejected(self):
        with self.assertRaises(CRCError):
            parse_frame(b"HX1|42|STOP|A1B2C3D4|0000\n")

    def test_lowercase_crc_is_rejected(self):
        with self.assertRaises(FrameFormatError):
            parse_frame(b"HX1|42|STOP|A1B2C3D4|972b\n")

    def test_wrong_prefix_is_rejected(self):
        with self.assertRaises(FrameFormatError):
            parse_frame(b"BAD|42|STOP|A1B2C3D4|0000\n")

    def test_surrounding_field_whitespace_is_rejected(self):
        body = b"HX1|42|STOP| A1B2C3D4"
        crc = crc16_ccitt_false(body)
        raw = body + ("|%04X\n" % crc).encode("ascii")

        with self.assertRaises(FrameFormatError):
            parse_frame(raw)


class SequenceTests(unittest.TestCase):
    def test_monotonic_sequence(self):
        self.assertTrue(sequence_is_newer(11, 10))
        self.assertFalse(sequence_is_newer(10, 10))
        self.assertFalse(sequence_is_newer(9, 10))

    def test_wraparound(self):
        self.assertTrue(sequence_is_newer(0, 0xFFFFFFFF))
        self.assertTrue(sequence_is_newer(1, 0xFFFFFFFF))

    def test_exact_half_range_is_not_newer(self):
        self.assertFalse(sequence_is_newer(0x80000000, 0))


class LineFramerTests(unittest.TestCase):
    def test_fragmented_input(self):
        frame = encode_frame(1, "ARM", "A1B2C3D4")

        framer = LineFramer()
        self.assertEqual(framer.feed(frame[:5]), [])
        self.assertEqual(framer.feed(frame[5:]), [frame])

    def test_multiple_frames(self):
        first = encode_frame(1, "ARM", "A1B2C3D4")
        second = encode_frame(2, "START", "A1B2C3D4")

        framer = LineFramer()
        self.assertEqual(framer.feed(first + second), [first, second])

    def test_oversized_line_is_discarded_and_recovers(self):
        valid = encode_frame(3, "STOP", "A1B2C3D4")
        oversized = b"X" * 513 + b"\n"

        framer = LineFramer()
        self.assertEqual(framer.feed(oversized + valid), [valid])
        self.assertEqual(framer.framing_errors, 1)


if __name__ == "__main__":
    unittest.main()
