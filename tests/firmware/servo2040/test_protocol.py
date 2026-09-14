"""Host-side tests for the Servo 2040 asymmetric HX1 protocol core."""

from __future__ import annotations

import struct
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
    parse_command_frame,
    parse_frame,
    sequence_is_newer,
)

SESSION = "A1B2C3D4"
JOINTS = tuple(range(-900, 900, 100))


class CRC16Tests(unittest.TestCase):
    def test_standard_check_vector(self):
        self.assertEqual(crc16_ccitt_false(b"123456789"), 0x29B1)

    def test_documented_legacy_stop_body(self):
        body = b"HX1|42|STOP|A1B2C3D4"
        self.assertEqual(crc16_ccitt_false(body), 0x972B)


class FrameTests(unittest.TestCase):
    def test_binary_stop_has_frozen_golden_vector(self):
        self.assertEqual(
            encode_frame(42, "STOP", SESSION).hex().upper(),
            "4858010700002A000000D4C3B2A1AD2E",
        )

    def test_stop_is_binary_and_round_trips(self):
        raw = encode_frame(42, "STOP", SESSION)
        self.assertEqual(len(raw), 16)
        self.assertEqual(raw[:2], b"HX")
        self.assertEqual(parse_frame(raw), (42, "STOP", (SESSION,)))

    def test_stage_is_52_bytes_and_fast_parser_returns_native_joints(self):
        raw = encode_frame(7, "STAGE", SESSION, *JOINTS)
        self.assertEqual(len(raw), 52)
        seq, message_type, fields = parse_command_frame(raw)
        self.assertEqual(seq, 7)
        self.assertEqual(message_type, "STAGE")
        self.assertEqual(fields[0], SESSION)
        self.assertEqual(fields[1:], JOINTS)
        self.assertTrue(all(isinstance(v, int) for v in fields[1:]))

    def test_target_is_56_bytes_and_period_is_native_int(self):
        raw = encode_frame(8, "TARGET", SESSION, 20, *JOINTS)
        self.assertEqual(len(raw), 56)
        seq, message_type, fields = parse_command_frame(raw)
        self.assertEqual((seq, message_type), (8, "TARGET"))
        self.assertEqual(fields[0], SESSION)
        self.assertEqual(fields[1], 20)
        self.assertIsInstance(fields[1], int)
        self.assertEqual(fields[2:], JOINTS)

    def test_heartbeat_uptime_is_native_int(self):
        raw = encode_frame(9, "HEARTBEAT", SESSION, 12345)
        _, _, fields = parse_command_frame(raw)
        self.assertEqual(fields, (SESSION, 12345))

    def test_generic_parser_stringifies_binary_fields_for_diagnostics(self):
        raw = encode_frame(8, "TARGET", SESSION, 20, *JOINTS)
        self.assertEqual(
            parse_frame(raw),
            (8, "TARGET", (SESSION, "20") + tuple(str(v) for v in JOINTS)),
        )

    def test_ascii_telemetry_round_trip_and_crlf(self):
        raw = encode_frame(7, "ACK", 6, "ARM")
        self.assertTrue(raw.endswith(b"\n"))
        self.assertEqual(parse_frame(raw), (7, "ACK", ("6", "ARM")))
        self.assertEqual(parse_frame(raw[:-1] + b"\r\n"), (7, "ACK", ("6", "ARM")))

    def test_binary_bad_crc_is_rejected(self):
        raw = bytearray(encode_frame(7, "STAGE", SESSION, *JOINTS))
        raw[-1] ^= 0x80
        with self.assertRaises(CRCError):
            parse_command_frame(raw)

    def test_binary_bad_length_is_rejected(self):
        raw = bytearray(encode_frame(7, "STAGE", SESSION, *JOINTS))
        # payload length is uint16 at bytes 4..5; claim 35 instead of 36.
        raw[4:6] = struct.pack("<H", 35)
        with self.assertRaises(FrameFormatError):
            parse_command_frame(raw)

    def test_ascii_bad_crc_is_rejected(self):
        with self.assertRaises(CRCError):
            parse_frame(b"HX1|42|ACK|1|STOP|0000\n")


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


class HybridFramerTests(unittest.TestCase):
    def test_fragmented_binary_frame(self):
        frame = encode_frame(1, "STAGE", SESSION, *JOINTS)
        framer = LineFramer()
        self.assertEqual(framer.feed(frame[:5]), [])
        self.assertEqual(framer.feed(frame[5:31]), [])
        self.assertEqual(framer.feed(frame[31:]), [frame])

    def test_multiple_binary_frames_are_returned_in_order(self):
        first = encode_frame(1, "ARM", SESSION)
        second = encode_frame(2, "TARGET", SESSION, 20, *JOINTS)
        framer = LineFramer()
        self.assertEqual(framer.feed(first + second), [first, second])

    def test_binary_payload_may_contain_newline_byte(self):
        joints = list(JOINTS)
        joints[0] = 10  # little-endian bytes include 0x0A
        frame = encode_frame(1, "STAGE", SESSION, *joints)
        self.assertIn(b"\n", frame)
        framer = LineFramer()
        self.assertEqual(framer.feed(frame), [frame])

    def test_ascii_diagnostic_frame_still_works(self):
        frame = encode_frame(1, "ACK", 0, "HELLO")
        framer = LineFramer()
        self.assertEqual(framer.feed(frame[:7]), [])
        self.assertEqual(framer.feed(frame[7:]), [frame])

    def test_garbage_prefix_resynchronizes_to_binary_magic(self):
        valid = encode_frame(3, "STOP", SESSION)
        framer = LineFramer()
        self.assertEqual(framer.feed(b"garbage" + valid), [valid])
        self.assertGreaterEqual(framer.framing_errors, 1)

    def test_fixed_length_header_corruption_does_not_consume_next_frame(self):
        bad = bytearray(encode_frame(1, "STAGE", SESSION, *JOINTS))
        bad[4:6] = struct.pack("<H", 35)
        valid = encode_frame(2, "STOP", SESSION)
        framer = LineFramer()
        frames = framer.feed(bytes(bad) + valid)
        self.assertIn(valid, frames)
        self.assertGreaterEqual(framer.framing_errors, 1)


if __name__ == "__main__":
    unittest.main()
