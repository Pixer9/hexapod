"""Tests for explicit LineFramer line abortion used by USB transport."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
FIRMWARE_SRC = REPO_ROOT / "firmware" / "servo2040" / "src"
sys.path.insert(0, str(FIRMWARE_SRC))

from hexapod_mcu.protocol import LineFramer, encode_frame  # noqa: E402


class LineFramerAbortTests(unittest.TestCase):
    def test_explicit_discard_drops_partial_line_and_recovers(self):
        framer = LineFramer()
        valid = encode_frame(2, "STOP", "A1B2C3D4")

        self.assertEqual(framer.feed(b"HX1|1|AR"), [])
        framer.discard_current_line()

        self.assertEqual(framer.feed(b"M|garbage\n" + valid), [valid])
        self.assertEqual(framer.framing_errors, 1)


if __name__ == "__main__":
    unittest.main()
