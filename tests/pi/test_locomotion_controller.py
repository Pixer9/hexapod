"""Tests for deterministic locomotion phase and transition control."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC = REPO_ROOT / "src"
ROBOT_CONFIG = REPO_ROOT / "config" / "robots" / "standard.json"
GAIT_CONFIG = REPO_ROOT / "config" / "locomotion" / "tripod.json"
CONTROLLER_CONFIG = (
    REPO_ROOT / "config" / "locomotion" / "controller.json"
)
sys.path.insert(0, str(SRC))

from hexapod.control import MotionCommand  # noqa: E402
from hexapod.kinematics import RobotKinematics  # noqa: E402
from hexapod.locomotion import (  # noqa: E402
    LocomotionController,
    LocomotionControllerConfig,
    LocomotionControllerConfigError,
    LocomotionControllerError,
    LocomotionMode,
    TripodGait,
    load_locomotion_controller_config,
    load_tripod_gait_config,
    parse_locomotion_controller_config,
)
from hexapod.model import load_robot_geometry  # noqa: E402


def assert_vec3_almost_equal(
    case: unittest.TestCase,
    actual,
    expected,
    places: int = 9,
):
    for actual_value, expected_value in zip(actual, expected):
        case.assertAlmostEqual(
            actual_value,
            expected_value,
            places=places,
        )


class LocomotionControllerConfigTests(unittest.TestCase):
    def test_loads_baseline_controller_config(self):
        config = load_locomotion_controller_config(CONTROLLER_CONFIG)

        self.assertEqual(config.schema_version, 1)
        self.assertEqual(
            config.controller_id,
            "locomotion-controller-v1",
        )
        self.assertEqual(config.start_blend_s, 0.45)
        self.assertEqual(config.stop_blend_s, 0.45)

    def test_rejects_negative_transition_duration(self):
        with self.assertRaises(LocomotionControllerConfigError):
            parse_locomotion_controller_config(
                {
                    "schema_version": 1,
                    "controller_id": "test",
                    "start_blend_s": -0.1,
                    "stop_blend_s": 0.45,
                }
            )

    def test_rejects_unknown_controller_config_keys(self):
        with self.assertRaises(LocomotionControllerConfigError):
            parse_locomotion_controller_config(
                {
                    "schema_version": 1,
                    "controller_id": "test",
                    "start_blend_s": 0.45,
                    "stop_blend_s": 0.45,
                    "mystery": True,
                }
            )


class LocomotionControllerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.geometry = load_robot_geometry(ROBOT_CONFIG)
        cls.gait_config = load_tripod_gait_config(GAIT_CONFIG)

    def make_controller(
        self,
        *,
        start_blend_s: float = 0.45,
        stop_blend_s: float = 0.45,
    ) -> LocomotionController:
        gait = TripodGait(self.geometry, self.gait_config)
        config = LocomotionControllerConfig(
            schema_version=1,
            controller_id="test-controller",
            start_blend_s=start_blend_s,
            stop_blend_s=stop_blend_s,
        )
        return LocomotionController(gait, config)

    def test_idle_zero_command_stays_flat_and_phase_frozen(self):
        controller = self.make_controller()

        frame = controller.step(MotionCommand.zero(), 10.0)

        self.assertEqual(frame.mode, LocomotionMode.IDLE)
        self.assertEqual(frame.global_phase, 0.0)
        self.assertEqual(frame.blend, 0.0)
        self.assertTrue(frame.trajectory_command.is_zero)

        for leg_name, anchor in controller.gait.stance_anchors_body_mm.items():
            assert_vec3_almost_equal(
                self,
                frame.gait_frame.foot_targets_body_mm[leg_name],
                anchor,
            )

    def test_start_transition_midpoint_is_half_blend(self):
        controller = self.make_controller()
        command = MotionCommand(vx_mm_s=80.0)

        controller.step(command, 0.0)
        frame = controller.step(command, 0.225)

        self.assertEqual(frame.mode, LocomotionMode.STARTING)
        self.assertAlmostEqual(frame.global_phase, 0.225)
        self.assertAlmostEqual(frame.blend, 0.5)
        self.assertEqual(frame.trajectory_command, command)

    def test_start_transition_reaches_moving(self):
        controller = self.make_controller()
        command = MotionCommand(vx_mm_s=80.0)

        frame = controller.step(command, 0.45)

        self.assertEqual(frame.mode, LocomotionMode.MOVING)
        self.assertAlmostEqual(frame.global_phase, 0.45)
        self.assertEqual(frame.blend, 1.0)

    def test_phase_advances_and_wraps_from_explicit_dt(self):
        controller = self.make_controller(
            start_blend_s=0.0,
            stop_blend_s=0.0,
        )
        command = MotionCommand(vx_mm_s=40.0)

        frame = controller.step(command, 1.25)

        self.assertEqual(frame.mode, LocomotionMode.MOVING)
        self.assertAlmostEqual(frame.global_phase, 0.25)
        self.assertEqual(frame.blend, 1.0)

    def test_normal_stop_latches_last_motion_command(self):
        controller = self.make_controller()
        command = MotionCommand(
            vx_mm_s=40.0,
            vy_mm_s=20.0,
            yaw_rate_deg_s=30.0,
        )

        controller.step(command, 0.45)
        start_stop = controller.step(MotionCommand.zero(), 0.0)

        self.assertEqual(start_stop.mode, LocomotionMode.STOPPING)
        self.assertEqual(start_stop.blend, 1.0)
        self.assertTrue(start_stop.requested_command.is_zero)
        self.assertEqual(start_stop.trajectory_command, command)

        midpoint = controller.step(MotionCommand.zero(), 0.225)

        self.assertEqual(midpoint.mode, LocomotionMode.STOPPING)
        self.assertAlmostEqual(midpoint.blend, 0.5)
        self.assertEqual(midpoint.trajectory_command, command)

    def test_stop_completion_resets_phase_and_flat_stance(self):
        controller = self.make_controller()
        command = MotionCommand(vx_mm_s=80.0)

        controller.step(command, 0.45)
        controller.step(MotionCommand.zero(), 0.225)
        frame = controller.step(MotionCommand.zero(), 0.225)

        self.assertEqual(frame.mode, LocomotionMode.IDLE)
        self.assertEqual(frame.global_phase, 0.0)
        self.assertEqual(frame.blend, 0.0)
        self.assertTrue(frame.trajectory_command.is_zero)

        for leg_name, anchor in controller.gait.stance_anchors_body_mm.items():
            assert_vec3_almost_equal(
                self,
                frame.gait_frame.foot_targets_body_mm[leg_name],
                anchor,
            )

    def test_restart_during_stop_reverses_envelope_without_reset(self):
        controller = self.make_controller()
        forward = MotionCommand(vx_mm_s=80.0)
        reverse = MotionCommand(vx_mm_s=-80.0)

        controller.step(forward, 0.45)
        stopping = controller.step(MotionCommand.zero(), 0.1125)
        blend_before = stopping.blend
        phase_before = stopping.global_phase

        restarted = controller.step(reverse, 0.0)

        self.assertEqual(restarted.mode, LocomotionMode.STARTING)
        self.assertAlmostEqual(restarted.blend, blend_before)
        self.assertAlmostEqual(restarted.global_phase, phase_before)
        self.assertEqual(restarted.trajectory_command, reverse)

    def test_time_partitioning_is_deterministic(self):
        command = MotionCommand(
            vx_mm_s=50.0,
            vy_mm_s=-20.0,
            yaw_rate_deg_s=15.0,
        )

        one_step = self.make_controller()
        split_steps = self.make_controller()

        frame_one = one_step.step(command, 0.3)

        frame_split = None
        for _ in range(3):
            frame_split = split_steps.step(command, 0.1)

        assert frame_split is not None
        self.assertEqual(frame_one.mode, frame_split.mode)
        self.assertAlmostEqual(
            frame_one.global_phase,
            frame_split.global_phase,
        )
        self.assertAlmostEqual(frame_one.blend, frame_split.blend)

        for leg_name in self.geometry.leg_order:
            assert_vec3_almost_equal(
                self,
                frame_one.gait_frame.foot_targets_body_mm[leg_name],
                frame_split.gait_frame.foot_targets_body_mm[leg_name],
            )

    def test_reset_returns_to_deterministic_idle(self):
        controller = self.make_controller()
        controller.step(MotionCommand(vx_mm_s=80.0), 0.3)

        controller.reset()

        self.assertEqual(controller.mode, LocomotionMode.IDLE)
        self.assertEqual(controller.global_phase, 0.0)
        self.assertEqual(controller.blend, 0.0)
        self.assertTrue(controller.trajectory_command.is_zero)

    def test_invalid_dt_is_rejected(self):
        controller = self.make_controller()

        for value in (-0.01, math.nan, math.inf):
            with self.subTest(value=value):
                with self.assertRaises(LocomotionControllerError):
                    controller.step(MotionCommand.zero(), value)

    def test_zero_duration_transitions_are_supported(self):
        controller = self.make_controller(
            start_blend_s=0.0,
            stop_blend_s=0.0,
        )

        moving = controller.step(
            MotionCommand(vx_mm_s=20.0),
            0.0,
        )
        self.assertEqual(moving.mode, LocomotionMode.MOVING)
        self.assertEqual(moving.blend, 1.0)

        idle = controller.step(MotionCommand.zero(), 0.0)
        self.assertEqual(idle.mode, LocomotionMode.IDLE)
        self.assertEqual(idle.blend, 0.0)

    def test_controller_outputs_remain_kinematically_solvable(self):
        controller = self.make_controller()
        kinematics = RobotKinematics(self.geometry)
        full_command = MotionCommand(
            vx_mm_s=80.0,
            vy_mm_s=60.0,
            yaw_rate_deg_s=90.0,
        )

        # Start and run for one second at 50 Hz.
        for _ in range(50):
            frame = controller.step(full_command, 0.02)
            solution = kinematics.solve(
                frame.gait_frame.foot_targets_body_mm
            )
            self.assertEqual(len(solution.logical_vector_deg), 18)

        # Normal stop blend back to deterministic flat stance.
        for _ in range(30):
            frame = controller.step(MotionCommand.zero(), 0.02)
            solution = kinematics.solve(
                frame.gait_frame.foot_targets_body_mm
            )
            self.assertEqual(len(solution.logical_vector_deg), 18)

        self.assertEqual(frame.mode, LocomotionMode.IDLE)
        self.assertEqual(frame.global_phase, 0.0)
        self.assertEqual(frame.blend, 0.0)


if __name__ == "__main__":
    unittest.main()
