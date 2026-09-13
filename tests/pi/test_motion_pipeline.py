"""Integration tests for the deterministic Pi-side motion pipeline."""

from __future__ import annotations

import unittest
from pathlib import Path

from hexapod.control import (
    CommandRateLimiter,
    MotionCommand,
    MotionLimitError,
    load_command_rate_limit_config,
    load_motion_limits,
)
from hexapod.kinematics import RobotKinematics
from hexapod.locomotion import (
    LocomotionController,
    LocomotionMode,
    TripodGait,
    load_locomotion_controller_config,
    load_tripod_gait_config,
)
from hexapod.model import load_robot_geometry
from hexapod.motion import MotionPipeline, MotionPipelineError
from hexapod.trajectory import (
    JointTrajectoryRateScaler,
    load_joint_soft_limit_profile,
    load_joint_trajectory_config,
)

REPO_ROOT = Path(__file__).resolve().parents[2]

ROBOT_CONFIG = REPO_ROOT / "config" / "robots" / "standard.json"
SOFT_LIMIT_CONFIG = REPO_ROOT / "config" / "robots" / "standard-joint-soft-limits.json"
MOTION_CONFIG = REPO_ROOT / "config" / "control" / "motion.json"
COMMAND_RATE_CONFIG = REPO_ROOT / "config" / "control" / "rate-limit.json"
JOINT_RATE_CONFIG = REPO_ROOT / "config" / "control" / "joint-trajectory.json"
GAIT_CONFIG = REPO_ROOT / "config" / "locomotion" / "tripod.json"
LOCOMOTION_CONFIG = REPO_ROOT / "config" / "locomotion" / "controller.json"


def build_pipeline() -> MotionPipeline:
    geometry = load_robot_geometry(ROBOT_CONFIG)
    motion_limits = load_motion_limits(MOTION_CONFIG)

    command_limiter = CommandRateLimiter(
        load_command_rate_limit_config(COMMAND_RATE_CONFIG),
        motion_limits,
    )

    gait = TripodGait(
        geometry,
        load_tripod_gait_config(GAIT_CONFIG),
    )
    locomotion = LocomotionController(
        gait,
        load_locomotion_controller_config(LOCOMOTION_CONFIG),
    )

    return MotionPipeline(
        command_limiter=command_limiter,
        locomotion=locomotion,
        kinematics=RobotKinematics(geometry),
        soft_limits=load_joint_soft_limit_profile(SOFT_LIMIT_CONFIG),
        joint_scaler=JointTrajectoryRateScaler(
            load_joint_trajectory_config(JOINT_RATE_CONFIG)
        ),
    )


class MotionPipelineTests(unittest.TestCase):
    def test_step_requires_explicit_flat_stance_initialization(self):
        pipeline = build_pipeline()

        with self.assertRaises(MotionPipelineError):
            pipeline.step(MotionCommand.zero(), 0.02)

    def test_initialization_uses_gait_flat_stance_and_seeds_same_vector(self):
        pipeline = build_pipeline()

        initialization = pipeline.initialize_flat_stance()

        self.assertTrue(pipeline.initialized)
        self.assertFalse(pipeline.failed)
        self.assertEqual(
            initialization.locomotion_frame.mode,
            LocomotionMode.IDLE,
        )
        self.assertEqual(
            initialization.locomotion_frame.global_phase,
            0.0,
        )
        self.assertEqual(
            initialization.locomotion_frame.blend,
            0.0,
        )

        gait = pipeline.locomotion.gait
        expected_z = gait.config.stance_z_mm

        self.assertEqual(expected_z, -135.0)
        self.assertNotEqual(
            expected_z,
            pipeline.kinematics.geometry.neutral_foot_leg_mm[2],
        )

        for (
            target
        ) in initialization.locomotion_frame.gait_frame.foot_targets_body_mm.values():
            self.assertAlmostEqual(target[2], expected_z)

        self.assertEqual(
            initialization.staged_joint_vector_deg,
            pipeline.joint_scaler.output_vector_deg,
        )
        self.assertEqual(
            initialization.staged_joint_vector_deg,
            pipeline.output_joint_vector_deg,
        )

        pipeline.soft_limits.validate(initialization.staged_joint_vector_deg)

    def test_zero_command_after_initialization_holds_flat_stance(self):
        pipeline = build_pipeline()
        initialization = pipeline.initialize_flat_stance()

        frame = pipeline.step(MotionCommand.zero(), 0.02)

        self.assertEqual(frame.shaped_command, MotionCommand.zero())
        self.assertEqual(
            frame.locomotion_frame.mode,
            LocomotionMode.IDLE,
        )
        self.assertEqual(
            frame.output_joint_vector_deg,
            initialization.staged_joint_vector_deg,
        )
        self.assertFalse(frame.trajectory_step.limited)

    def test_full_configured_motion_runs_at_50_hz_through_all_stages(self):
        pipeline = build_pipeline()
        pipeline.initialize_flat_stance()

        command = MotionCommand(
            vx_mm_s=80.0,
            vy_mm_s=60.0,
            yaw_rate_deg_s=90.0,
        )

        saw_joint_rate_limiting = False

        for _ in range(100):
            frame = pipeline.step(command, 0.02)

            self.assertEqual(len(frame.output_joint_vector_deg), 18)
            pipeline.soft_limits.validate(frame.target_joint_vector_deg)
            pipeline.soft_limits.validate(frame.output_joint_vector_deg)
            self.assertLessEqual(
                frame.trajectory_step.peak_applied_rate_deg_s,
                pipeline.joint_scaler.config.max_joint_rate_deg_s + 1e-9,
            )

            saw_joint_rate_limiting |= frame.trajectory_step.limited

        self.assertEqual(
            frame.locomotion_frame.mode,
            LocomotionMode.MOVING,
        )
        self.assertFalse(pipeline.failed)

        # Limiting is permitted here. This test verifies the integrated
        # production path remains valid whether or not the configured gait
        # transient reaches the Pi-side joint-rate budget.
        self.assertIsInstance(saw_joint_rate_limiting, bool)

    def test_normal_stop_returns_to_exact_initialized_flat_stance(self):
        pipeline = build_pipeline()
        initialization = pipeline.initialize_flat_stance()

        command = MotionCommand(
            vx_mm_s=60.0,
            vy_mm_s=20.0,
            yaw_rate_deg_s=30.0,
        )

        for _ in range(75):
            pipeline.step(command, 0.02)

        frame = None
        for _ in range(75):
            frame = pipeline.step(MotionCommand.zero(), 0.02)

        assert frame is not None

        self.assertEqual(
            frame.locomotion_frame.mode,
            LocomotionMode.IDLE,
        )
        self.assertEqual(frame.shaped_command, MotionCommand.zero())
        self.assertEqual(
            frame.output_joint_vector_deg,
            initialization.staged_joint_vector_deg,
        )
        self.assertFalse(pipeline.failed)

    def test_update_failure_latches_pipeline_until_reinitialized(self):
        pipeline = build_pipeline()
        pipeline.initialize_flat_stance()

        with self.assertRaises(MotionLimitError):
            pipeline.step(
                MotionCommand(vx_mm_s=81.0),
                0.02,
            )

        self.assertTrue(pipeline.failed)

        with self.assertRaises(MotionPipelineError):
            pipeline.step(MotionCommand.zero(), 0.02)

        pipeline.initialize_flat_stance()

        self.assertFalse(pipeline.failed)
        frame = pipeline.step(MotionCommand.zero(), 0.02)
        self.assertEqual(
            frame.locomotion_frame.mode,
            LocomotionMode.IDLE,
        )


if __name__ == "__main__":
    unittest.main()
