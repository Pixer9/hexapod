"""Tests for deterministic normal MotionCommand rate limiting."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC = REPO_ROOT / "src"
MOTION_CONFIG = REPO_ROOT / "config" / "control" / "motion.json"
RATE_CONFIG = REPO_ROOT / "config" / "control" / "rate-limit.json"
ROBOT_CONFIG = REPO_ROOT / "config" / "robots" / "standard.json"
GAIT_CONFIG = REPO_ROOT / "config" / "locomotion" / "tripod.json"
LOCOMOTION_CONFIG = (
    REPO_ROOT / "config" / "locomotion" / "controller.json"
)
sys.path.insert(0, str(SRC))

from hexapod.control import (  # noqa: E402
    CommandRateLimitError,
    CommandRateLimiter,
    MotionCommand,
    MotionLimitError,
    load_command_rate_limit_config,
    load_motion_limits,
    parse_command_rate_limit_config,
)
from hexapod.kinematics import RobotKinematics  # noqa: E402
from hexapod.locomotion import (  # noqa: E402
    LocomotionController,
    TripodGait,
    load_locomotion_controller_config,
    load_tripod_gait_config,
)
from hexapod.model import load_robot_geometry  # noqa: E402


class CommandRateLimitConfigTests(unittest.TestCase):
    def test_loads_baseline_config(self):
        config = load_command_rate_limit_config(RATE_CONFIG)

        self.assertEqual(config.schema_version, 1)
        self.assertEqual(
            config.profile_id,
            "standard-command-rate-v1",
        )
        self.assertEqual(config.translation_accel_mm_s2, 260.0)
        self.assertEqual(config.translation_decel_mm_s2, 520.0)
        self.assertEqual(config.yaw_accel_deg_s2, 280.0)
        self.assertEqual(config.yaw_decel_deg_s2, 700.0)

    def test_rejects_nonpositive_rate(self):
        with self.assertRaises(CommandRateLimitError):
            parse_command_rate_limit_config(
                {
                    "schema_version": 1,
                    "profile_id": "test",
                    "units": {
                        "translation_acceleration":
                            "millimeter_per_second_squared",
                        "yaw_acceleration":
                            "degree_per_second_squared",
                    },
                    "limits": {
                        "translation_accel_mm_s2": 0.0,
                        "translation_decel_mm_s2": 520.0,
                        "yaw_accel_deg_s2": 280.0,
                        "yaw_decel_deg_s2": 700.0,
                    },
                }
            )

    def test_rejects_unknown_config_key(self):
        with self.assertRaises(CommandRateLimitError):
            parse_command_rate_limit_config(
                {
                    "schema_version": 1,
                    "profile_id": "test",
                    "units": {
                        "translation_acceleration":
                            "millimeter_per_second_squared",
                        "yaw_acceleration":
                            "degree_per_second_squared",
                    },
                    "limits": {
                        "translation_accel_mm_s2": 260.0,
                        "translation_decel_mm_s2": 520.0,
                        "yaw_accel_deg_s2": 280.0,
                        "yaw_decel_deg_s2": 700.0,
                        "mystery": 1.0,
                    },
                }
            )


class CommandRateLimiterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_command_rate_limit_config(RATE_CONFIG)
        cls.limits = load_motion_limits(MOTION_CONFIG)

    def make_limiter(self) -> CommandRateLimiter:
        return CommandRateLimiter(self.config, self.limits)

    def test_initial_output_is_zero(self):
        self.assertTrue(self.make_limiter().output.is_zero)

    def test_zero_dt_does_not_move(self):
        limiter = self.make_limiter()

        result = limiter.step(
            MotionCommand(vx_mm_s=80.0),
            0.0,
        )

        self.assertTrue(result.is_zero)

    def test_translation_acceleration_preserves_target_direction(self):
        limiter = self.make_limiter()

        result = limiter.step(
            MotionCommand(vx_mm_s=80.0, vy_mm_s=60.0),
            0.1,
        )

        # Target vector magnitude is 100 mm/s. A 260 mm/s^2 acceleration
        # budget for 0.1 s produces a 26 mm/s vector in the same 4:3 ratio.
        self.assertAlmostEqual(result.vx_mm_s, 20.8)
        self.assertAlmostEqual(result.vy_mm_s, 15.6)
        self.assertAlmostEqual(
            math.hypot(result.vx_mm_s, result.vy_mm_s),
            26.0,
        )

    def test_translation_reaches_target_without_overshoot(self):
        limiter = self.make_limiter()
        target = MotionCommand(vx_mm_s=40.0, vy_mm_s=-20.0)

        result = limiter.step(target, 1.0)

        self.assertEqual(result, target)

    def test_nonzero_collinear_slowdown_uses_deceleration_rate(self):
        limiter = self.make_limiter()
        limiter.reset(
            MotionCommand(vx_mm_s=80.0)
        )

        result = limiter.step(
            MotionCommand(vx_mm_s=40.0),
            0.05,
        )

        # 520 mm/s^2 for 0.05 s removes 26 mm/s.
        self.assertAlmostEqual(
            result.vx_mm_s,
            54.0,
        )
        self.assertEqual(
            result.vy_mm_s,
            0.0,
        )

    def test_translation_deceleration_preserves_current_direction(self):
        limiter = self.make_limiter()
        limiter.reset(
            MotionCommand(vx_mm_s=48.0, vy_mm_s=36.0)
        )

        result = limiter.step(MotionCommand.zero(), 0.05)

        # Initial magnitude is 60. A 520 mm/s^2 decel for 0.05 s removes 26,
        # leaving magnitude 34 while retaining the original 4:3 direction.
        self.assertAlmostEqual(
            math.hypot(result.vx_mm_s, result.vy_mm_s),
            34.0,
        )
        self.assertAlmostEqual(
            result.vx_mm_s / result.vy_mm_s,
            4.0 / 3.0,
        )

    def test_translation_reversal_decelerates_before_crossing_zero(self):
        limiter = self.make_limiter()
        limiter.reset(MotionCommand(vx_mm_s=80.0))

        result = limiter.step(
            MotionCommand(vx_mm_s=-80.0),
            0.1,
        )

        self.assertAlmostEqual(result.vx_mm_s, 28.0)
        self.assertEqual(result.vy_mm_s, 0.0)

    def test_translation_reversal_uses_leftover_time_after_zero(self):
        limiter = self.make_limiter()
        limiter.reset(MotionCommand(vx_mm_s=80.0))

        result = limiter.step(
            MotionCommand(vx_mm_s=-80.0),
            0.2,
        )

        # 80/520 s removes the old velocity. The remaining time accelerates
        # toward -80 at 260 mm/s^2, producing exactly -12 mm/s.
        self.assertAlmostEqual(result.vx_mm_s, -12.0)
        self.assertEqual(result.vy_mm_s, 0.0)

    def test_yaw_acceleration_uses_configured_rate(self):
        limiter = self.make_limiter()

        result = limiter.step(
            MotionCommand(yaw_rate_deg_s=90.0),
            0.1,
        )

        self.assertAlmostEqual(result.yaw_rate_deg_s, 28.0)

    def test_yaw_deceleration_uses_configured_rate(self):
        limiter = self.make_limiter()
        limiter.reset(MotionCommand(yaw_rate_deg_s=90.0))

        result = limiter.step(MotionCommand.zero(), 0.1)

        self.assertAlmostEqual(result.yaw_rate_deg_s, 20.0)

    def test_yaw_reversal_decelerates_then_accelerates(self):
        limiter = self.make_limiter()
        limiter.reset(MotionCommand(yaw_rate_deg_s=90.0))

        result = limiter.step(
            MotionCommand(yaw_rate_deg_s=-90.0),
            0.2,
        )

        # 90/700 s to zero, then the remaining time accelerates negative.
        expected = -280.0 * (0.2 - 90.0 / 700.0)
        self.assertAlmostEqual(result.yaw_rate_deg_s, expected)

    def test_reset_can_hard_set_state(self):
        limiter = self.make_limiter()
        state = MotionCommand(
            vx_mm_s=10.0,
            vy_mm_s=-5.0,
            yaw_rate_deg_s=12.0,
        )

        self.assertEqual(limiter.reset(state), state)
        self.assertEqual(limiter.output, state)
        self.assertTrue(limiter.reset().is_zero)

    def test_invalid_dt_is_rejected(self):
        limiter = self.make_limiter()

        for value in (-0.01, math.nan, math.inf):
            with self.subTest(value=value):
                with self.assertRaises(CommandRateLimitError):
                    limiter.step(MotionCommand.zero(), value)

    def test_out_of_envelope_target_is_rejected(self):
        limiter = self.make_limiter()

        with self.assertRaises(MotionLimitError):
            limiter.step(
                MotionCommand(vx_mm_s=81.0),
                0.02,
            )

    def test_turn_is_time_partition_deterministic(self):
        target = MotionCommand(
            vx_mm_s=0.0,
            vy_mm_s=60.0,
        )

        one_step = self.make_limiter()
        split = self.make_limiter()

        initial = MotionCommand(
            vx_mm_s=80.0,
            vy_mm_s=0.0,
        )

        one_step.reset(initial)
        split.reset(initial)

        result_one = one_step.step(target, 0.2)

        split.step(target, 0.1)
        result_split = split.step(target, 0.1)

        self.assertAlmostEqual(
            result_one.vx_mm_s,
            result_split.vx_mm_s,
        )
        self.assertAlmostEqual(
            result_one.vy_mm_s,
            result_split.vy_mm_s,
        )

        self.assertAlmostEqual(
            result_one.vx_mm_s,
            38.4,
        )
        self.assertAlmostEqual(
            result_one.vy_mm_s,
            31.2,
        )

    def test_reversal_is_time_partition_deterministic(self):
        target = MotionCommand(vx_mm_s=-80.0)

        one_step = self.make_limiter()
        split = self.make_limiter()
        one_step.reset(MotionCommand(vx_mm_s=80.0))
        split.reset(MotionCommand(vx_mm_s=80.0))

        result_one = one_step.step(target, 0.2)
        split.step(target, 0.1)
        result_split = split.step(target, 0.1)

        self.assertAlmostEqual(
            result_one.vx_mm_s,
            result_split.vx_mm_s,
        )
        self.assertAlmostEqual(
            result_one.vy_mm_s,
            result_split.vy_mm_s,
        )
        self.assertAlmostEqual(
            result_one.yaw_rate_deg_s,
            result_split.yaw_rate_deg_s,
        )

    def test_full_pipeline_remains_kinematically_solvable(self):
        geometry = load_robot_geometry(ROBOT_CONFIG)
        gait = TripodGait(
            geometry,
            load_tripod_gait_config(GAIT_CONFIG),
        )
        locomotion = LocomotionController(
            gait,
            load_locomotion_controller_config(
                LOCOMOTION_CONFIG
            ),
        )
        kinematics = RobotKinematics(geometry)
        limiter = self.make_limiter()

        target = MotionCommand(
            vx_mm_s=80.0,
            vy_mm_s=60.0,
            yaw_rate_deg_s=90.0,
        )

        for _ in range(60):
            shaped = limiter.step(target, 0.02)
            frame = locomotion.step(shaped, 0.02)
            solution = kinematics.solve(
                frame.gait_frame.foot_targets_body_mm
            )
            self.assertEqual(len(solution.logical_vector_deg), 18)

        for _ in range(60):
            shaped = limiter.step(MotionCommand.zero(), 0.02)
            frame = locomotion.step(shaped, 0.02)
            solution = kinematics.solve(
                frame.gait_frame.foot_targets_body_mm
            )
            self.assertEqual(len(solution.logical_vector_deg), 18)

        self.assertTrue(limiter.output.is_zero)


if __name__ == "__main__":
    unittest.main()
