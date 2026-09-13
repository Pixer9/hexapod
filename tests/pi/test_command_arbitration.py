"""Tests for freshness-aware normal command-source arbitration."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC = REPO_ROOT / "src"
CONFIG = REPO_ROOT / "config" / "control" / "motion.json"
sys.path.insert(0, str(SRC))

from hexapod.control import (  # noqa: E402
    CommandArbiter,
    CommandSample,
    MotionCommand,
    MotionLimitError,
    MotionSource,
    load_motion_limits,
)


def sample(
    source: MotionSource,
    vx: float,
    received: float,
    expires: float,
) -> CommandSample:
    return CommandSample(
        source=source,
        command=MotionCommand(vx_mm_s=vx),
        received_at_s=received,
        expires_at_s=expires,
    )


class CommandArbitrationTests(unittest.TestCase):
    def setUp(self):
        self.limits = load_motion_limits(CONFIG)
        self.arbiter = CommandArbiter(self.limits)

    def test_no_sources_produces_zero_idle_command(self):
        selected = self.arbiter.select(10.0)

        self.assertEqual(selected.source, MotionSource.IDLE)
        self.assertTrue(selected.command.is_zero)
        self.assertEqual(selected.age_s, 0.0)

    def test_priority_is_ds4_then_web_then_autonomy(self):
        self.arbiter.publish(sample(MotionSource.AUTONOMY, 10.0, 1.0, 20.0))
        self.arbiter.publish(sample(MotionSource.WEB, 20.0, 1.0, 20.0))
        self.arbiter.publish(sample(MotionSource.DS4, 30.0, 1.0, 20.0))

        selected = self.arbiter.select(5.0)

        self.assertEqual(selected.source, MotionSource.DS4)
        self.assertEqual(selected.command.vx_mm_s, 30.0)

    def test_zero_ds4_command_still_owns_control_while_fresh(self):
        self.arbiter.publish(sample(MotionSource.AUTONOMY, 50.0, 1.0, 20.0))
        self.arbiter.publish(sample(MotionSource.DS4, 0.0, 2.0, 20.0))

        selected = self.arbiter.select(3.0)

        self.assertEqual(selected.source, MotionSource.DS4)
        self.assertTrue(selected.command.is_zero)

    def test_expired_ds4_falls_back_to_fresh_web(self):
        self.arbiter.publish(sample(MotionSource.WEB, 20.0, 1.0, 20.0))
        self.arbiter.publish(sample(MotionSource.DS4, 30.0, 1.0, 5.0))

        selected = self.arbiter.select(5.0)

        self.assertEqual(selected.source, MotionSource.WEB)
        self.assertEqual(selected.command.vx_mm_s, 20.0)

    def test_exact_expiry_is_stale(self):
        command = sample(MotionSource.DS4, 10.0, 1.0, 5.0)

        self.assertTrue(command.is_fresh(4.999))
        self.assertFalse(command.is_fresh(5.0))

    def test_command_is_not_fresh_before_received_timestamp(self):
        command = sample(MotionSource.DS4, 10.0, 5.0, 10.0)

        self.assertFalse(command.is_fresh(4.999))

    def test_clear_removes_source_immediately(self):
        self.arbiter.publish(sample(MotionSource.DS4, 10.0, 1.0, 20.0))
        self.arbiter.clear(MotionSource.DS4)

        self.assertEqual(
            self.arbiter.select(2.0).source,
            MotionSource.IDLE,
        )

    def test_out_of_order_publication_is_rejected(self):
        self.arbiter.publish(sample(MotionSource.DS4, 10.0, 5.0, 10.0))

        with self.assertRaises(ValueError):
            self.arbiter.publish(sample(MotionSource.DS4, 20.0, 4.0, 10.0))

    def test_out_of_envelope_source_command_is_rejected(self):
        with self.assertRaises(MotionLimitError):
            self.arbiter.publish(sample(MotionSource.WEB, 81.0, 1.0, 10.0))

    def test_idle_cannot_be_published(self):
        with self.assertRaises(ValueError):
            CommandSample(
                source=MotionSource.IDLE,
                command=MotionCommand.zero(),
                received_at_s=1.0,
                expires_at_s=2.0,
            )

    def test_invalid_lifetime_is_rejected(self):
        with self.assertRaises(ValueError):
            sample(MotionSource.DS4, 10.0, 5.0, 5.0)

    def test_selected_command_reports_age(self):
        self.arbiter.publish(sample(MotionSource.DS4, 10.0, 2.5, 10.0))

        selected = self.arbiter.select(4.0)

        self.assertAlmostEqual(selected.age_s, 1.5)


if __name__ == "__main__":
    unittest.main()
