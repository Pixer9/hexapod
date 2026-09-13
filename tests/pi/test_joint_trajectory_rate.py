"""Tests for global canonical joint-trajectory rate scaling."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC = REPO_ROOT / "src"

TRAJECTORY_CONFIG = (
    REPO_ROOT / "config" / "control" / "joint-trajectory.json"
)
MOTION_CONFIG = REPO_ROOT / "config" / "control" / "motion.json"
RATE_CONFIG = REPO_ROOT / "config" / "control" / "rate-limit.json"
ROBOT_CONFIG = REPO_ROOT / "config" / "robots" / "standard.json"
GAIT_CONFIG = REPO_ROOT / "config" / "locomotion" / "tripod.json"
LOCOMOTION_CONFIG = (
    REPO_ROOT / "config" / "locomotion" / "controller.json"
)

sys.path.insert(0, str(SRC))

from hexapod.control import (  # noqa: E402
    CommandRateLimiter,
    MotionCommand,
    load_command_rate_limit_config,
    load_motion_limits,
)
from hexapod.kinematics import LOGICAL_JOINT_COUNT, RobotKinematics  # noqa: E402
from hexapod.locomotion import (  # noqa: E402
    LocomotionController,
    TripodGait,
    load_locomotion_controller_config,
    load_tripod_gait_config,
)
from hexapod.model import load_robot_geometry  # noqa: E402
from hexapod.trajectory import (  # noqa: E402
    JointTrajectoryError,
    JointTrajectoryRateScaler,
    load_joint_trajectory_config,
    parse_joint_trajectory_config,
)


def vector_with(*pairs: tuple[int, float]) -> tuple[float, ...]:
    values = [0.0] * LOGICAL_JOINT_COUNT
    for index, value in pairs:
        values[index] = value
    return tuple(values)


class JointTrajectoryConfigTests(unittest.TestCase):
    def test_loads_baseline_config(self):
        config = load_joint_trajectory_config(TRAJECTORY_CONFIG)

        self.assertEqual(config.schema_version, 1)
        self.assertEqual(
            config.profile_id,
            "standard-joint-trajectory-v1",
        )
        self.assertEqual(config.max_joint_rate_deg_s, 250.0)

    def test_rejects_nonpositive_rate(self):
        with self.assertRaises(JointTrajectoryError):
            parse_joint_trajectory_config(
                {
                    "schema_version": 1,
                    "profile_id": "test",
                    "units": {
                        "joint_rate": "degree_per_second",
                    },
                    "limits": {
                        "max_joint_rate_deg_s": 0.0,
                    },
                }
            )

    def test_rejects_wrong_units(self):
        with self.assertRaises(JointTrajectoryError):
            parse_joint_trajectory_config(
                {
                    "schema_version": 1,
                    "profile_id": "test",
                    "units": {
                        "joint_rate": "centidegree_per_second",
                    },
                    "limits": {
                        "max_joint_rate_deg_s": 250.0,
                    },
                }
            )

    def test_rejects_unknown_config_key(self):
        with self.assertRaises(JointTrajectoryError):
            parse_joint_trajectory_config(
                {
                    "schema_version": 1,
                    "profile_id": "test",
                    "units": {
                        "joint_rate": "degree_per_second",
                    },
                    "limits": {
                        "max_joint_rate_deg_s": 250.0,
                        "mystery": 1.0,
                    },
                }
            )


class JointTrajectoryRateScalerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_joint_trajectory_config(TRAJECTORY_CONFIG)

    def make_scaler(self) -> JointTrajectoryRateScaler:
        return JointTrajectoryRateScaler(self.config)

    def test_initial_state_is_explicitly_uninitialized(self):
        scaler = self.make_scaler()

        self.assertFalse(scaler.initialized)
        with self.assertRaises(JointTrajectoryError):
            _ = scaler.output_vector_deg

    def test_step_before_seed_is_rejected(self):
        scaler = self.make_scaler()

        with self.assertRaises(JointTrajectoryError):
            scaler.step(vector_with((0, 1.0)), 0.02)

    def test_seed_sets_exact_current_command(self):
        scaler = self.make_scaler()
        initial = tuple(float(index) for index in range(LOGICAL_JOINT_COUNT))

        result = scaler.seed(initial)

        self.assertTrue(scaler.initialized)
        self.assertEqual(result, initial)
        self.assertEqual(scaler.output_vector_deg, initial)

    def test_seed_rejects_wrong_vector_length(self):
        scaler = self.make_scaler()

        with self.assertRaises(JointTrajectoryError):
            scaler.seed((0.0,) * (LOGICAL_JOINT_COUNT - 1))

    def test_target_rejects_nonfinite_joint(self):
        scaler = self.make_scaler()
        scaler.seed((0.0,) * LOGICAL_JOINT_COUNT)
        target = list((0.0,) * LOGICAL_JOINT_COUNT)
        target[5] = math.nan

        with self.assertRaises(JointTrajectoryError):
            scaler.step(target, 0.02)

    def test_identical_target_is_not_limited(self):
        scaler = self.make_scaler()
        initial = vector_with((0, 10.0), (7, -20.0))
        scaler.seed(initial)

        step = scaler.step(initial, 0.02)

        self.assertFalse(step.limited)
        self.assertEqual(step.scale, 1.0)
        self.assertIsNone(step.limiting_joint_index)
        self.assertEqual(step.max_target_delta_deg, 0.0)
        self.assertEqual(step.required_duration_s, 0.0)
        self.assertEqual(step.peak_applied_rate_deg_s, 0.0)
        self.assertEqual(step.output_vector_deg, initial)

    def test_exact_budget_reaches_target(self):
        scaler = self.make_scaler()
        scaler.seed((0.0,) * LOGICAL_JOINT_COUNT)

        # 250 deg/s * 0.02 s = 5 degrees.
        target = vector_with((3, 5.0))
        step = scaler.step(target, 0.02)

        self.assertFalse(step.limited)
        self.assertEqual(step.scale, 1.0)
        self.assertEqual(step.output_vector_deg, target)
        self.assertAlmostEqual(step.peak_applied_rate_deg_s, 250.0)

    def test_over_budget_scales_entire_vector_uniformly(self):
        scaler = self.make_scaler()
        scaler.seed((0.0,) * LOGICAL_JOINT_COUNT)

        target = vector_with(
            (0, 10.0),
            (1, -4.0),
            (2, 2.0),
        )
        step = scaler.step(target, 0.02)

        self.assertTrue(step.limited)
        self.assertAlmostEqual(step.scale, 0.5)
        self.assertEqual(step.limiting_joint_index, 0)
        self.assertAlmostEqual(step.output_vector_deg[0], 5.0)
        self.assertAlmostEqual(step.output_vector_deg[1], -2.0)
        self.assertAlmostEqual(step.output_vector_deg[2], 1.0)
        self.assertAlmostEqual(step.peak_applied_rate_deg_s, 250.0)

    def test_global_scaling_preserves_joint_delta_ratios(self):
        scaler = self.make_scaler()
        initial = vector_with(
            (0, 20.0),
            (1, -10.0),
            (2, 5.0),
        )
        target = vector_with(
            (0, 40.0),
            (1, -2.0),
            (2, 1.0),
        )
        scaler.seed(initial)

        step = scaler.step(target, 0.02)

        applied = tuple(
            new - old
            for old, new in zip(initial, step.output_vector_deg)
        )
        requested = tuple(
            new - old
            for old, new in zip(initial, target)
        )

        self.assertTrue(step.limited)
        for applied_delta, requested_delta in zip(applied, requested):
            if requested_delta == 0.0:
                self.assertEqual(applied_delta, 0.0)
            else:
                self.assertAlmostEqual(
                    applied_delta / requested_delta,
                    step.scale,
                )

    def test_zero_dt_holds_current_state(self):
        scaler = self.make_scaler()
        initial = vector_with((0, 5.0))
        scaler.seed(initial)

        step = scaler.step(vector_with((0, 15.0)), 0.0)

        self.assertTrue(step.limited)
        self.assertEqual(step.scale, 0.0)
        self.assertEqual(step.output_vector_deg, initial)
        self.assertEqual(step.peak_applied_rate_deg_s, 0.0)
        self.assertAlmostEqual(step.required_duration_s, 10.0 / 250.0)

    def test_invalid_dt_is_rejected(self):
        scaler = self.make_scaler()
        scaler.seed((0.0,) * LOGICAL_JOINT_COUNT)

        for value in (-0.01, math.nan, math.inf):
            with self.subTest(value=value):
                with self.assertRaises(JointTrajectoryError):
                    scaler.step((0.0,) * LOGICAL_JOINT_COUNT, value)

    def test_fixed_target_is_time_partition_deterministic(self):
        target = vector_with((0, 20.0), (1, -8.0))

        one_step = self.make_scaler()
        split = self.make_scaler()
        zero = (0.0,) * LOGICAL_JOINT_COUNT
        one_step.seed(zero)
        split.seed(zero)

        result_one = one_step.step(target, 0.04)

        split.step(target, 0.02)
        result_split = split.step(target, 0.02)

        for value_one, value_split in zip(
            result_one.output_vector_deg,
            result_split.output_vector_deg,
        ):
            self.assertAlmostEqual(value_one, value_split)

    def test_applied_rate_never_exceeds_budget(self):
        scaler = self.make_scaler()
        scaler.seed(
            tuple(
                -50.0 + index * 3.0
                for index in range(LOGICAL_JOINT_COUNT)
            )
        )
        target = tuple(
            80.0 - index * 4.0
            for index in range(LOGICAL_JOINT_COUNT)
        )

        previous = scaler.output_vector_deg
        step = scaler.step(target, 0.013)

        measured_peak = max(
            abs(new - old) / 0.013
            for old, new in zip(previous, step.output_vector_deg)
        )
        self.assertLessEqual(measured_peak, 250.0 + 1e-9)
        self.assertAlmostEqual(
            step.peak_applied_rate_deg_s,
            measured_peak,
        )

    def test_limiter_index_reports_largest_requested_delta(self):
        scaler = self.make_scaler()
        scaler.seed((0.0,) * LOGICAL_JOINT_COUNT)

        step = scaler.step(
            vector_with(
                (4, 7.0),
                (11, -12.0),
                (17, 9.0),
            ),
            0.02,
        )

        self.assertTrue(step.limited)
        self.assertEqual(step.limiting_joint_index, 11)
        self.assertEqual(step.max_target_delta_deg, 12.0)
        self.assertAlmostEqual(step.required_duration_s, 12.0 / 250.0)

    def test_full_motion_pipeline_respects_joint_rate_budget(self):
        geometry = load_robot_geometry(ROBOT_CONFIG)
        gait = TripodGait(
            geometry,
            load_tripod_gait_config(GAIT_CONFIG),
        )
        locomotion = LocomotionController(
            gait,
            load_locomotion_controller_config(LOCOMOTION_CONFIG),
        )
        command_limiter = CommandRateLimiter(
            load_command_rate_limit_config(RATE_CONFIG),
            load_motion_limits(MOTION_CONFIG),
        )
        kinematics = RobotKinematics(geometry)
        scaler = self.make_scaler()

        idle = locomotion.step(MotionCommand.zero(), 0.0)
        idle_solution = kinematics.solve(
            idle.gait_frame.foot_targets_body_mm
        )
        scaler.seed(idle_solution.logical_vector_deg)

        full_command = MotionCommand(
            vx_mm_s=80.0,
            vy_mm_s=60.0,
            yaw_rate_deg_s=90.0,
        )

        previous = scaler.output_vector_deg
        for _ in range(80):
            shaped = command_limiter.step(full_command, 0.02)
            locomotion_frame = locomotion.step(shaped, 0.02)
            solution = kinematics.solve(
                locomotion_frame.gait_frame.foot_targets_body_mm
            )
            step = scaler.step(solution.logical_vector_deg, 0.02)

            measured_peak = max(
                abs(new - old) / 0.02
                for old, new in zip(previous, step.output_vector_deg)
            )
            self.assertLessEqual(measured_peak, 250.0 + 1e-8)
            previous = step.output_vector_deg

        for _ in range(80):
            shaped = command_limiter.step(MotionCommand.zero(), 0.02)
            locomotion_frame = locomotion.step(shaped, 0.02)
            solution = kinematics.solve(
                locomotion_frame.gait_frame.foot_targets_body_mm
            )
            step = scaler.step(solution.logical_vector_deg, 0.02)

            measured_peak = max(
                abs(new - old) / 0.02
                for old, new in zip(previous, step.output_vector_deg)
            )
            self.assertLessEqual(measured_peak, 250.0 + 1e-8)
            previous = step.output_vector_deg


if __name__ == "__main__":
    unittest.main()
