"""Tests for deterministic tripod gait trajectory generation."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC = REPO_ROOT / "src"
ROBOT_CONFIG = REPO_ROOT / "config" / "robots" / "standard.json"
GAIT_CONFIG = REPO_ROOT / "config" / "locomotion" / "tripod.json"
sys.path.insert(0, str(SRC))

from hexapod.control import MotionCommand  # noqa: E402
from hexapod.kinematics import RobotKinematics  # noqa: E402
from hexapod.locomotion import (  # noqa: E402
    GaitConfigError,
    GaitError,
    TRIPOD_A,
    TRIPOD_B,
    TripodGait,
    load_tripod_gait_config,
    parse_tripod_gait_config,
    tripod_phase_for_leg,
)
from hexapod.model import CANONICAL_LEG_ORDER, load_robot_geometry  # noqa: E402


class TripodGaitConfigTests(unittest.TestCase):
    def test_standard_config_loads(self):
        config = load_tripod_gait_config(GAIT_CONFIG)
        self.assertEqual(config.gait_id, "tripod-v1")
        self.assertEqual(config.cycle_hz, 1.0)
        self.assertEqual(config.duty_factor, 0.5)
        self.assertEqual(config.step_height_mm, 30.0)
        self.assertEqual(config.stance_z_mm, -135.0)
        self.assertEqual(config.max_foot_offset_mm, 50.0)

    def test_duty_factor_below_tripod_support_floor_is_rejected(self):
        with self.assertRaises(GaitConfigError):
            parse_tripod_gait_config(
                {
                    "schema_version": 1,
                    "gait_id": "bad",
                    "cycle_hz": 1.0,
                    "duty_factor": 0.49,
                    "step_height_mm": 70.0,
                    "stance_z_mm": -135.0,
                    "max_foot_offset_mm": 50.0,
                }
            )

    def test_unknown_config_keys_are_rejected(self):
        with self.assertRaises(GaitConfigError):
            parse_tripod_gait_config(
                {
                    "schema_version": 1,
                    "gait_id": "bad",
                    "cycle_hz": 1.0,
                    "duty_factor": 0.5,
                    "step_height_mm": 70.0,
                    "stance_z_mm": -135.0,
                    "max_foot_offset_mm": 50.0,
                    "mystery": 123,
                }
            )


class TripodPhaseTests(unittest.TestCase):
    def test_tripod_membership_covers_all_legs_once(self):
        self.assertEqual(set(TRIPOD_A) | set(TRIPOD_B), set(CANONICAL_LEG_ORDER))
        self.assertFalse(set(TRIPOD_A) & set(TRIPOD_B))
        self.assertEqual(len(TRIPOD_A), 3)
        self.assertEqual(len(TRIPOD_B), 3)

    def test_tripod_b_is_half_cycle_out_of_phase(self):
        for leg in TRIPOD_A:
            self.assertAlmostEqual(tripod_phase_for_leg(0.125, leg), 0.125)
        for leg in TRIPOD_B:
            self.assertAlmostEqual(tripod_phase_for_leg(0.125, leg), 0.625)

    def test_unknown_leg_is_rejected(self):
        with self.assertRaises(GaitError):
            tripod_phase_for_leg(0.0, "XX")


class TripodTrajectoryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.geometry = load_robot_geometry(ROBOT_CONFIG)
        cls.config = load_tripod_gait_config(GAIT_CONFIG)
        cls.gait = TripodGait(cls.geometry, cls.config)
        cls.kinematics = RobotKinematics(cls.geometry)

    def test_zero_command_returns_flat_stance_at_any_phase(self):
        for phase in (0.0, 0.125, 0.25, 0.5, 0.875):
            frame = self.gait.sample(MotionCommand.zero(), phase)
            self.assertEqual(frame.blend, 0.0)
            for leg, target in frame.foot_targets_body_mm.items():
                anchor = self.gait.stance_anchors_body_mm[leg]
                self.assertEqual(target, anchor)

    def test_zero_blend_returns_flat_stance(self):
        frame = self.gait.sample(
            MotionCommand(vx_mm_s=80.0),
            0.25,
            blend=0.0,
        )
        for leg, target in frame.foot_targets_body_mm.items():
            self.assertEqual(target, self.gait.stance_anchors_body_mm[leg])

    def test_forward_command_sweeps_tripod_a_front_to_rear(self):
        command = MotionCommand(vx_mm_s=80.0)
        touchdown = self.gait.sample(command, 0.0)
        near_liftoff = self.gait.sample(command, 0.5 - 1e-9)

        for leg in TRIPOD_A:
            anchor_x = self.gait.stance_anchors_body_mm[leg][0]
            self.assertGreater(touchdown.foot_targets_body_mm[leg][0], anchor_x)
            self.assertLess(near_liftoff.foot_targets_body_mm[leg][0], anchor_x)

    def test_left_strafe_sweeps_tripod_a_left_to_right(self):
        command = MotionCommand(vy_mm_s=60.0)
        touchdown = self.gait.sample(command, 0.0)
        near_liftoff = self.gait.sample(command, 0.5 - 1e-9)

        for leg in TRIPOD_A:
            anchor_y = self.gait.stance_anchors_body_mm[leg][1]
            self.assertGreater(touchdown.foot_targets_body_mm[leg][1], anchor_y)
            self.assertLess(near_liftoff.foot_targets_body_mm[leg][1], anchor_y)

    def test_tripods_alternate_stance_and_swing(self):
        frame = self.gait.sample(MotionCommand(vx_mm_s=40.0), 0.25)
        self.assertTrue(all(frame.leg_states[leg].in_stance for leg in TRIPOD_A))
        self.assertTrue(all(not frame.leg_states[leg].in_stance for leg in TRIPOD_B))

        frame = self.gait.sample(MotionCommand(vx_mm_s=40.0), 0.75)
        self.assertTrue(all(not frame.leg_states[leg].in_stance for leg in TRIPOD_A))
        self.assertTrue(all(frame.leg_states[leg].in_stance for leg in TRIPOD_B))

    def test_swing_midpoint_reaches_configured_step_height(self):
        # At global phase 0.25, tripod B local phase is 0.75: midpoint of
        # swing for duty_factor=0.5.
        frame = self.gait.sample(MotionCommand(vx_mm_s=40.0), 0.25)
        for leg in TRIPOD_B:
            z = frame.foot_targets_body_mm[leg][2]
            self.assertAlmostEqual(
                z,
                self.config.stance_z_mm + self.config.step_height_mm,
                places=10,
            )

    def test_horizontal_excursion_is_globally_limited(self):
        frame = self.gait.sample(
            MotionCommand(
                vx_mm_s=80.0,
                vy_mm_s=60.0,
                yaw_rate_deg_s=90.0,
            ),
            0.0,
        )
        self.assertLess(frame.command_scale, 1.0)

        max_offset = 0.0
        for leg, target in frame.foot_targets_body_mm.items():
            anchor = self.gait.stance_anchors_body_mm[leg]
            offset = math.hypot(target[0] - anchor[0], target[1] - anchor[1])
            max_offset = max(max_offset, offset)
        self.assertLessEqual(max_offset, self.config.max_foot_offset_mm + 1e-9)
        self.assertAlmostEqual(max_offset, self.config.max_foot_offset_mm, places=9)

    def test_positive_yaw_touchdown_offsets_follow_ccw_body_point_motion(self):
        command = MotionCommand(yaw_rate_deg_s=45.0)

        # RF is in Tripod A and touches down at global phase 0.0.
        rf_frame = self.gait.sample(command, 0.0)
        rf_anchor = self.gait.stance_anchors_body_mm["RF"]
        rf = rf_frame.foot_targets_body_mm["RF"]
        self.assertGreater(rf[0] - rf_anchor[0], 0.0)
        self.assertGreater(rf[1] - rf_anchor[1], 0.0)

        # LF is in Tripod B and touches down at global phase 0.5.
        lf_frame = self.gait.sample(command, 0.5)
        lf_anchor = self.gait.stance_anchors_body_mm["LF"]
        lf = lf_frame.foot_targets_body_mm["LF"]
        self.assertLess(lf[0] - lf_anchor[0], 0.0)
        self.assertGreater(lf[1] - lf_anchor[1], 0.0)

    def test_blend_scales_horizontal_and_vertical_trajectory(self):
        command = MotionCommand(vx_mm_s=40.0)
        full = self.gait.sample(command, 0.25, blend=1.0)
        half = self.gait.sample(command, 0.25, blend=0.5)

        for leg in CANONICAL_LEG_ORDER:
            anchor = self.gait.stance_anchors_body_mm[leg]
            for axis in (0, 1, 2):
                full_delta = full.foot_targets_body_mm[leg][axis] - anchor[axis]
                half_delta = half.foot_targets_body_mm[leg][axis] - anchor[axis]
                self.assertAlmostEqual(half_delta, 0.5 * full_delta, places=10)

    def test_invalid_blend_is_rejected(self):
        with self.assertRaises(GaitError):
            self.gait.sample(MotionCommand(vx_mm_s=10.0), 0.0, blend=1.1)

    def test_full_command_samples_remain_kinematically_solvable(self):
        command = MotionCommand(
            vx_mm_s=80.0,
            vy_mm_s=60.0,
            yaw_rate_deg_s=90.0,
        )
        for index in range(40):
            phase = index / 40.0
            frame = self.gait.sample(command, phase)
            solution = self.kinematics.solve(frame.foot_targets_body_mm)
            self.assertEqual(len(solution.logical_vector_deg), 18)


if __name__ == "__main__":
    unittest.main()
