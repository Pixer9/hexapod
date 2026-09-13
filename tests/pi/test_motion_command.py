"""Tests for the shared robot motion-command contract."""

from __future__ import annotations

import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC = REPO_ROOT / "src"
CONFIG = REPO_ROOT / "config" / "control" / "motion.json"
sys.path.insert(0, str(SRC))

from hexapod.control import (  # noqa: E402
    MotionCommand,
    MotionCommandError,
    MotionConfigError,
    MotionLimitError,
    MotionLimits,
    load_motion_limits,
)


class MotionCommandTests(unittest.TestCase):
    def test_zero_command(self):
        command = MotionCommand.zero()

        self.assertTrue(command.is_zero)
        self.assertEqual(command.vx_mm_s, 0.0)
        self.assertEqual(command.vy_mm_s, 0.0)
        self.assertEqual(command.yaw_rate_deg_s, 0.0)

    def test_nonfinite_command_is_rejected(self):
        for bad in (math.inf, -math.inf, math.nan):
            with self.subTest(value=bad):
                with self.assertRaises(MotionCommandError):
                    MotionCommand(vx_mm_s=bad)

    def test_standard_limits_preserve_v4_controller_envelope(self):
        limits = load_motion_limits(CONFIG)

        self.assertEqual(limits.max_vx_mm_s, 80.0)
        self.assertEqual(limits.max_vy_mm_s, 60.0)
        self.assertEqual(limits.max_yaw_rate_deg_s, 90.0)

    def test_limits_are_inclusive(self):
        limits = load_motion_limits(CONFIG)

        command = MotionCommand(
            vx_mm_s=-80.0,
            vy_mm_s=60.0,
            yaw_rate_deg_s=-90.0,
        )

        self.assertIs(limits.validate(command), command)

    def test_limit_violation_is_rejected_not_clamped(self):
        limits = load_motion_limits(CONFIG)

        with self.assertRaises(MotionLimitError):
            limits.validate(MotionCommand(vx_mm_s=80.001))

    def test_bad_units_are_rejected(self):
        data = json.loads(CONFIG.read_text(encoding="utf-8"))
        data["units"]["linear_velocity"] = "meter_per_second"

        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "bad.json"
            path.write_text(json.dumps(data), encoding="utf-8")

            with self.assertRaises(MotionConfigError):
                load_motion_limits(path)

    def test_nonpositive_limit_is_rejected(self):
        with self.assertRaises(MotionCommandError):
            MotionLimits(
                max_vx_mm_s=0.0,
                max_vy_mm_s=60.0,
                max_yaw_rate_deg_s=90.0,
            )


if __name__ == "__main__":
    unittest.main()
