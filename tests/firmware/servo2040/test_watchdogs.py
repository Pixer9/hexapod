"""Host-side tests for wrap-safe watchdog helpers."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
FIRMWARE_SRC = REPO_ROOT / "firmware" / "servo2040" / "src"
sys.path.insert(0, str(FIRMWARE_SRC))

from hexapod_mcu.watchdogs import age_ms, is_fresh, is_timed_out  # noqa: E402


class WatchdogHelperTests(unittest.TestCase):
    def test_missing_timestamp_has_age_minus_one(self):
        self.assertEqual(age_ms(100, None), -1)

    def test_age_is_elapsed_time(self):
        self.assertEqual(age_ms(1250, 1000), 250)

    def test_freshness_is_inclusive(self):
        self.assertTrue(is_fresh(2000, 0, 2000))
        self.assertFalse(is_fresh(2001, 0, 2000))

    def test_timeout_triggers_at_threshold(self):
        self.assertFalse(is_timed_out(199, 0, 200))
        self.assertTrue(is_timed_out(200, 0, 200))


if __name__ == "__main__":
    unittest.main()
